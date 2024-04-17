/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/comms/altitude.h>
#include <cola2_lib/comms/elapsed_time.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/Mission.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/Recovery.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <cstdint>
#include <ctime>
#include <string>

#include "cppystruct/cppystruct.h"

namespace
{
const auto FMT =
    PY_STRING("<1s1d2d2f1I2B");            //!< format of the message transmitted from the vehicle {id, time, lat, lon,
                                           //!< depth, yaw/accuracy, status_code/command, altitude, elapsed_time }
const int SIZE = pystruct::calcsize(FMT);  //!< size of the serialized message
}  // namespace

/**
 * \brief Commands that are not part of cola2_msgs::RecoveryAction and are directly executed by cola2_comms.
 */
enum CustomCommands
{
  START_MISSION = 10,  //!< start default mission
  RESET_TIMEOUT = 20   //!< reset vehicle timeout
};

/**
 * \class That hadles the interchange of data between the surface USBL and the vehicle where this node is running.
 */
class CommsNode
{
private:
  /**
   * \brief Configuration loaded from ROS param server.
   */
  struct Config
  {
    double custom_max_time;       //!< max time since last custom input reception to still send it
    bool usbl_safe_always_on;     //!< check that vehicle continuously receives modem messages
    double send_to_modem_period;  //!< time between messages sent to modem
    int max_message_size;         //!< max message size (defined by modem)
    std::string identifier;       //!< letter that identifies the sender
  };
  Config config_;  //!< configuration params

  // NodeHandle
  ros::NodeHandle nh_;  //!< node handler
  // Publishers
  ros::Publisher pub_custom_output_;  //!< binarized user specific output
  ros::Publisher pub_to_modem_;       //!< message to be sent by the modem
  ros::Publisher pub_usbl_;           //!< usbl update received from modem
  // Subscribers
  ros::Subscriber sub_from_modem_;     //!< data received by the modem
  ros::Subscriber sub_safety_status_;  //!< current safety status to get status code
  ros::Subscriber sub_navigation_;     //!< vehicle navigation
  ros::Subscriber sub_elapsed_time_;   //!< watchdog elapsed time
  ros::Subscriber sub_custom_input_;   //!< binarized user specific input
  // ServiceClients
  ros::ServiceClient srv_start_mission_;  //!< service to call mission start
  ros::ServiceClient srv_reset_timeout_;  //!< service to reset vehicle timeout
  // ServiceServers
  ros::ServiceServer srv_reload_params_;          //!< reload config of this node
  ros::ServiceServer srv_reset_recovery_action_;  //!< reset recovery action to ok
  // Timer
  ros::Timer timer_;  //!< timer to send messages to modem

  // Diagnostics
  struct
  {
    double modem_time = 0.0;  //!< time since last message from modem
    decltype(diagnostic_msgs::DiagnosticStatus::level) diagnostics_level =
        diagnostic_msgs::DiagnosticStatus::OK;  //!< last level reported by the modem
    decltype(cola2_msgs::RecoveryAction::error_level) recovery_action =
        cola2_msgs::RecoveryAction::NONE;  //!< last recovery action called
  } diagnostics_;
  cola2::ros::DiagnosticHelper diag_help_;  //!< to create diagnostics

  // Saves from callbacks
  std::string last_message_ = "";                         //!< last received message
  bool get_modem_time_ = false;                           //!< flag to know if modem_time_ must be updated
  std::string custom_ = "";                               //!< custom extra message received
  double custom_time_ = ros::Time::now().toSec();         //!< time of last custom message
  uint32_t status_code_ = 0;                              //!< status code received
  int32_t elapsed_time_ = -1;                             //!< elapsed time received (init as invalid/not received)
  cola2_msgs::NavSts navigation_ = cola2_msgs::NavSts();  //!< navigation received
  bool init_navigation_ = false;                          //!< navigation arrived

  // Functions
  /**
   * \brief Load params from ROS param server.
   */
  bool loadParams();

  // Servers
  /**
   * \brief Service to reload the params of this node.
   * \param req Request
   * \param res Response
   * \return Success on the param reloading
   */
  bool srvReloadParams(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief Server to reset last_recovery_action to OK.
   *
   * Useful when aborting by modem and want to disable the abort once on surface with wifi connection.
   *
   * @param req
   * @param res
   * @return
   */
  bool srvResetRecoveryAction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // Callbacks
  /**
   * \brief Callback on the serialized data obtained from the modem.
   * \param msg Raw serialized string obtained from modem communications.
   */
  void cbkFromModem(const std_msgs::String &msg);
  /**
   * \brief Callback that listens to the custom input (optional) that will be attached with the basic message.
   */
  void cbkCustomInput(const std_msgs::String &msg);
  /**
   * \brief Callback to obtain the status code from SafetySupervisor to be communicated to surface.
   */
  void cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatus &msg);
  /**
   * \brief Callback to obtain vehicle navigation to be communicated to surface.
   */
  void cbkNavigation(const cola2_msgs::NavSts &msg);
  /**
   * \brief Callback to obtain elapsed time to be communicated to surface.
   */
  void cbkElapsedTime(const std_msgs::Int32 &msg);

  // Timer callback
  /**
   * \brief Timer to keep sending data to the modem that should go to surface.
   */
  void sendMessage(const ros::TimerEvent &event);

public:
  /**
   * Constructor.
   */
  CommsNode();
  /**
   * Destructor.
   */
  ~CommsNode();
};

CommsNode::CommsNode() : nh_("~"), diag_help_(nh_, "comms", cola2::ros::getUnresolvedNodeName())
{
  // Load params from param server
  loadParams();

  // USBL data timeout
  if (config_.usbl_safe_always_on)
  {
    get_modem_time_ = true;
    diagnostics_.modem_time = ros::Time::now().toSec();
  }

  // Publishers
  // clang-format off
  pub_to_modem_ = nh_.advertise<std_msgs::String>("to_modem", 1);
  pub_custom_output_ = nh_.advertise<std_msgs::String>("custom_output", 1);
  pub_usbl_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(cola2::ros::getNamespace() + "/navigator/usbl", 1);
  // clang-format on

  // Init services
  srv_reload_params_ = nh_.advertiseService("reload_params", &CommsNode::srvReloadParams, this);
  srv_reset_recovery_action_ = nh_.advertiseService("reset_recovery_action", &CommsNode::srvResetRecoveryAction, this);

  // Subscribers
  // clang-format off
  sub_from_modem_ = nh_.subscribe("from_modem", 1, &CommsNode::cbkFromModem, this);
  sub_custom_input_ = nh_.subscribe("custom_input", 1, &CommsNode::cbkCustomInput, this);
  sub_safety_status_ = nh_.subscribe(cola2::ros::getNamespace() + "/safety_supervisor/status", 1, &CommsNode::cbkSafetyStatus, this);
  sub_navigation_ = nh_.subscribe(cola2::ros::getNamespace() + "/navigator/navigation", 1, &CommsNode::cbkNavigation, this);
  sub_elapsed_time_ = nh_.subscribe(cola2::ros::getNamespace() + "/watchdog_timer/elapsed_time", 1, &CommsNode::cbkElapsedTime, this);
  // clang-format on

  // Service clients
  const std::string sname_start = cola2::ros::getNamespace() + "/captain/enable_mission";
  srv_start_mission_ = nh_.serviceClient<cola2_msgs::Mission>(sname_start);
  const std::string sname_timeout = cola2::ros::getNamespace() + "/watchdog_timer/reset_timeout";
  srv_reset_timeout_ = nh_.serviceClient<std_srvs::Trigger>(sname_timeout);
  // All service names
  const std::string snames = sname_start + " and " + sname_timeout;
  while (ros::ok())
  {
    if (srv_start_mission_.waitForExistence(ros::Duration(5.0)) &&
        srv_reset_timeout_.waitForExistence(ros::Duration(5.0)))
    {
      break;
    }
    ROS_INFO_STREAM("Waiting for client to services: " << snames);
  }

  // Timer
  timer_ = nh_.createTimer(ros::Duration(config_.send_to_modem_period), &CommsNode::sendMessage, this);

  // Diagnostics
  diag_help_.setFrequencyBufferTimeLimit(120.0);
  diag_help_.setFrequencyBufferMinData(3);
  diag_help_.setEnabled(true);

  ROS_INFO("Initialized");
}

CommsNode::~CommsNode()
{
}

bool CommsNode::loadParams()
{
  // Load from ROS param server
  // clang-format off
  cola2::ros::getParam("~custom_max_time", config_.custom_max_time, 1.0);
  cola2::ros::getParam("~usbl_safe_always_on", config_.usbl_safe_always_on);
  cola2::ros::getParam("~send_to_modem_period", config_.send_to_modem_period, 0.5);
  cola2::ros::getParam("~max_message_size", config_.max_message_size);
  cola2::ros::getParam("~identifier", config_.identifier);
  // clang-format on

  // Show loaded config
  ROS_INFO("Loaded config from param server");
  ROS_INFO("===============================");
  ROS_INFO("     custom_max_time: %.2f", config_.custom_max_time);
  ROS_INFO(" usbl_safe_always_on: %d  ", config_.usbl_safe_always_on);
  ROS_INFO("send_to_modem_period: %.2f", config_.send_to_modem_period);
  ROS_INFO("    max_message_size: %d  ", config_.max_message_size);
  ROS_INFO("          identifier: %s  ", config_.identifier.c_str());

  // Checks
  ROS_ASSERT(config_.max_message_size >= SIZE);  // enough message space for basic message
  ROS_ASSERT(config_.identifier.size() == 1);    // single char identifier

  // Return
  return true;
}

bool CommsNode::srvReloadParams(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
  res.success = loadParams();
  return res.success;
}
bool CommsNode::srvResetRecoveryAction(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
  diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnostics_.recovery_action = cola2_msgs::RecoveryAction::NONE;
  diagnostics_.modem_time = ros::Time::now().toSec();
  res.success = true;
  ROS_INFO("called srvResetRecoveryAction()");
  return res.success;
}

void CommsNode::cbkFromModem(const std_msgs::String &msg)
{
  // Message is big enough
  if (msg.data.size() < SIZE)
  {
    ROS_WARN("Invalid message length %zu (recv) != %d (fmt)", msg.data.size(), SIZE);
  }
  // Is not exactly the previous message
  if (last_message_.compare(msg.data) == 0)
  {
    return;
  }
  last_message_ = msg.data;
  diagnostics_.modem_time = ros::Time::now().toSec();
  get_modem_time_ = true;  // from the first modem message, keep updating time for safety

  // Unpack basic message
  std::string basic = msg.data.substr(0, SIZE);
  auto [id, tim, lat, lon, depth, accuracy, command, dummy_altitude, dummy_elapsed_time] = pystruct::unpack(FMT, basic);
  (void)dummy_altitude;      // Suppress unused variable warning
  (void)dummy_elapsed_time;  // Suppress unused variable warning

  // If it comes from usbl
  if (id.compare(std::string("U")) == 0)
  {
    // Correct reception
    diag_help_.reportValidData();

    // If it is a valid message
    if ((tim != 0.0) && (lat != 0.0))
    {
      // Construct USBL update
      geometry_msgs::PoseWithCovarianceStamped usbl;
      usbl.header.frame_id = cola2::ros::getNamespaceNoInitialDash() + "/modem";
      usbl.header.stamp = ros::Time(tim);
      usbl.pose.pose.position.x = lat;
      usbl.pose.pose.position.y = lon;
      usbl.pose.pose.position.z = depth;
      usbl.pose.covariance[0] = accuracy * accuracy;
      usbl.pose.covariance[7] = usbl.pose.covariance[0];
      usbl.pose.covariance[14] = usbl.pose.covariance[0];
      // Publish
      pub_usbl_.publish(usbl);
    }
    // Check the command
    diagnostics_.recovery_action = cola2_msgs::RecoveryAction::NONE;
    switch (command)
    {
      case cola2_msgs::RecoveryAction::NONE:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::OK;
        break;
      }
      case cola2_msgs::RecoveryAction::INFORMATIVE:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::OK;
        break;
      }
      case cola2_msgs::RecoveryAction::ABORT:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::WARN;
        diagnostics_.recovery_action = cola2_msgs::RecoveryAction::ABORT;
        ROS_WARN("Abort");
        break;
      }
      case cola2_msgs::RecoveryAction::ABORT_AND_SURFACE:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::WARN;
        diagnostics_.recovery_action = cola2_msgs::RecoveryAction::ABORT_AND_SURFACE;
        ROS_WARN("Abort and surface");
        break;
      }
      case cola2_msgs::RecoveryAction::EMERGENCY_SURFACE:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::WARN;
        diagnostics_.recovery_action = cola2_msgs::RecoveryAction::EMERGENCY_SURFACE;
        ROS_WARN("Emergency surface");
        break;
      }
      case cola2_msgs::RecoveryAction::DROP_WEIGHT:
      {
        ROS_WARN("Drop weight");
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::WARN;
        diagnostics_.recovery_action = cola2_msgs::RecoveryAction::DROP_WEIGHT;
        break;
      }
      case CustomCommands::START_MISSION:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::OK;
        ROS_WARN("Start mission");
        cola2_msgs::MissionRequest req;
        cola2_msgs::MissionResponse res;
        srv_start_mission_.call(req, res);
        break;
      }
      case CustomCommands::RESET_TIMEOUT:
      {
        diagnostics_.diagnostics_level = diagnostic_msgs::DiagnosticStatus::OK;
        ROS_WARN("Reset timeout");
        std_srvs::Trigger trigger;
        srv_reset_timeout_.call(trigger);
        break;
      }
      default:
      {
        ROS_ERROR("Unrecognized command from surface %d", command);
      }
    }
  }

  // Rest of message is custom message
  if (msg.data.size() > SIZE)
  {
    std_msgs::String custom;
    custom.data = msg.data.substr(SIZE, msg.data.size() - SIZE);
    pub_custom_output_.publish(custom);
  }
}

void CommsNode::cbkCustomInput(const std_msgs::String &msg)
{
  custom_ = msg.data;
  custom_time_ = ros::Time::now().toSec();
}
void CommsNode::cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatus &msg)
{
  status_code_ = msg.status_code;
}
void CommsNode::cbkNavigation(const cola2_msgs::NavSts &msg)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(msg))
  {
    return;
  }

  navigation_ = msg;
  init_navigation_ = true;
  // Avoid modem errors when vehicle is on surface and we are updating modem_time_
  if (get_modem_time_ && (navigation_.position.depth < 1.0))
  {
    diagnostics_.modem_time = ros::Time::now().toSec();
  }
}

void CommsNode::cbkElapsedTime(const std_msgs::Int32 &msg)
{
  elapsed_time_ = msg.data;
}

void CommsNode::sendMessage(const ros::TimerEvent &event)
{
  // Navigation available
  if (!init_navigation_)
  {
    diag_help_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "navigation not init");
    diag_help_.addKeyValue("modem_recovery_action", std::to_string(diagnostics_.recovery_action));
    if (get_modem_time_)
    {
      const double tim = ros::Time::now().toSec();
      diag_help_.addKeyValue("last_modem_data", tim - diagnostics_.modem_time);
    }
    diag_help_.publish();
    return;
  }
  // Encode altitude and elapsed time
  const uint8_t enc_altitude = altitudeEncode(navigation_.altitude);
  const uint8_t enc_elapsed_time = elapsedTimeEncode(elapsed_time_);
  // Create message
  auto packed =
      pystruct::pack(FMT, config_.identifier, navigation_.header.stamp.toSec(), navigation_.global_position.latitude,
                     navigation_.global_position.longitude, navigation_.position.depth, navigation_.orientation.yaw,
                     status_code_, enc_altitude, enc_elapsed_time);
  // Convert to string
  std_msgs::String msg;
  msg.data = std::string(std::begin(packed), std::end(packed));
  // Add custom data if not too old
  if ((event.current_real.toSec() - custom_time_) < config_.custom_max_time)
  {
    // Check that it fits
    if ((SIZE + static_cast<int>(custom_.size()) <= config_.max_message_size))
    {
      msg.data += custom_;
    }
    else
    {
      ROS_WARN("Ignoring custom message of size %zu because it doesn't fit", custom_.size());
      ROS_WARN("Max custom message size is %d", config_.max_message_size - SIZE);
    }
  }
  // Send message
  pub_to_modem_.publish(msg);
  // Check that we are receiving
  if (get_modem_time_)
  {
    const double tim = ros::Time::now().toSec();
    diag_help_.addKeyValue("last_modem_data", tim - diagnostics_.modem_time);
  }

  diag_help_.setLevelAndMessage(diagnostics_.diagnostics_level);
  diag_help_.addKeyValue("modem_recovery_action", std::to_string(diagnostics_.recovery_action));
  diag_help_.publish();
}

int main(int argc, char **argv)
{
  // Init
  ros::init(argc, argv, "comms");
  CommsNode node;
  ros::spin();  // spin until architecture stops
  return EXIT_SUCCESS;
}
