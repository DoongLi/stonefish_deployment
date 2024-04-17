/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/only_thruster_allocator.h>
#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_lib/utils/angles.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/serviceclient_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <cola2_msgs/GoalDescriptor.h>
#include <cola2_msgs/MaxJoyVelocity.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/Setpoints.h>
#include <cola2_msgs/WorldWaypointReq.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

// These define maximum velocities and turning rates for open loop teleoperation
static constexpr double OPEN_LOOP_MAX_SURGE_VEL = 1.0;  //!< Maximum open loop surge velocity.
static constexpr double OPEN_LOOP_MAX_SWAY_VEL = 0.4;   //!< Maximum open loop sway velocity.
static constexpr double OPEN_LOOP_MAX_HEAVE_VEL = 0.4;  //!< Maximum open loop heave velocity.
static constexpr double OPEN_LOOP_MAX_YAW_RATE = 0.6;   //!< Maximum open loop yaw turning rate.

/**
 * @brief We need a small wrapper class because the methods we want to access are protected.
 */
class ThrusterAllocatorWrapper : public OnlyThrusterAllocator
{
public:
  /**
   * @brief Constructor of the class.
   *
   * @param n_thrusters Number of thrusters.
   */
  explicit ThrusterAllocatorWrapper(unsigned int n_thrusters) : OnlyThrusterAllocator(n_thrusters)
  {
  }

  /**
   * @brief This method converts wrench to thruster forces.
   *
   * @param input Input wrench.
   * @return Returns the corresponding thruster forces.
   */
  std::vector<double> wrenchToThrusterForcesAsVector(const std::vector<double>& input)
  {
    Eigen::VectorXd wrench(input.size());
    for (std::size_t i = 0; i < input.size(); ++i)
    {
      wrench(i) = input[i];
    }
    const Eigen::VectorXd thruster_forces = wrenchToThrusterForces(wrench);
    std::vector<double> output(thruster_forces.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(thruster_forces.size()); ++i)
    {
      output[i] = thruster_forces(i);
    }
    return output;
  }

  /**
   * @brief This method converts thruster forces to thruster setpoints.
   *
   * @param input Input thruster forces.
   * @return Returns the corresponding thruster setpoints.
   */
  std::vector<double> forcesToSetpointsAsVector(const std::vector<double>& input)
  {
    Eigen::VectorXd thruster_forces(input.size());
    for (std::size_t i = 0; i < input.size(); ++i)
    {
      thruster_forces(i) = input[i];
    }
    const Eigen::VectorXd setpoint = forceToSetpoint(thruster_forces);
    std::vector<double> output(setpoint.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(setpoint.size()); ++i)
    {
      output[i] = setpoint(i);
    }
    return output;
  }
};

/**
 * @brief This class receives a joy message and generates a world_waypoint_req or a body_velocity_req.
 *
 * The joy message always have the same structure. The axis contain the value for pose and twist:
 *   --> axis: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
 * While the buttons decide if an axis is controlled in pose or in twist:
 *   --> buttons: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
 */
class Teleoperation
{
private:
  // ROS
  ros::NodeHandle nh_;                          //!< ROS node handle.
  ros::Publisher pub_bvr_;                      //!< This publishes a body velocity request.
  ros::Publisher pub_wwr_;                      //!< This publishes a world waypoint request.
  ros::Publisher pub_check_joystick_;           //!< This publishes the teleoperation ACK.
  ros::Publisher pub_thruster_setpoints_;       //!< This publishes thruster setpoints.
  ros::Subscriber sub_ack_;                     //!< Subscriber to the teleoperation ACK.
  ros::Subscriber sub_output_;                  //!< Subscriber to the teleoperation output.
  ros::Subscriber sub_nav_;                     //!< Subscriber to the robot navigation.
  ros::ServiceServer srv_set_max_joy_vel_;      //!< Service to set the maximum velocities.
  ros::ServiceServer srv_set_joy_axes_to_vel_;  //!< Service to set axes to control velocity.
  ros::ServiceServer srv_reload_params_;        //!< Service to reload params from the ROS param server.
  ros::ServiceServer srv_enable_thrusters_;     //!< Service to enable thrusters.
  ros::ServiceServer srv_disable_thrusters_;    //!< Service to disable thrusters.
  ros::Timer timer_slow_;                       //!< Timer at slow rate.
  ros::Timer timer_fast_;                       //!< Timer at fast rate.
  std::string ns_;                              //!< ROS namespace of this node.
  cola2::ros::DiagnosticHelper diagnostic_;     //!< Diagnostic helper.

  // ACK control
  double last_map_ack_;  //!< Time stamp in seconds of the last ACK message.
  bool map_ack_init_;    //!< It is set to true when a ACK is received.
  bool map_ack_alive_;   //!< Keeps track of the ACK.
  int seq_;              //!< Always increasing sequence number.

  // Last pose from the navigation
  double last_navigation_;         //!< Time stamp in seconds of the last valid navigation message.
  std::vector<double> last_pose_;  //!< Last pose from the navigation.
  bool has_navigation_;            //!< True if navigation received at least once.

  // These control the ranges of velocity and position
  std::vector<double> max_vel_;             //!< Array of maximum velocities for each DoF.
  std::vector<double> min_vel_;             //!< Array of minimum velocities for each DoF.
  std::vector<double> max_pos_;             //!< Array of maximum position/orientation for each DoF.
  std::vector<double> min_pos_;             //!< Array of minimum position/orientation for each DoF.
  std::vector<double> base_pose_;           //!< Pose from which the min and max poses will be computed.
  bool actualize_base_pose_;                //!< When true, base pose is updated when pose control is enabled.
  std::vector<bool> pose_controlled_axis_;  //!< This array keeps track of which DoF are pose controlled.

  // Reset keep position feature
  double last_nonzero_command_time_;  //!< Time stamp in seconds of the last nonzero command.
  bool reset_keep_position_called_;   //!< It is set to false when a a keep position is needed.

  // The following variables are for open loop teleoperation
  double max_wrench_surge_;  //!< Maximum surge force.
  double max_wrench_sway_;   //!< Maximum sway force.
  double max_wrench_heave_;  //!< Maximum heave force.
  double max_wrench_yaw_;    //!< Maximum yaw torque.
  Poly poly_surge_;          //!< Open loop force model for the surge DoF.
  Poly poly_sway_;           //!< Open loop force model for the sway DoF.
  Poly poly_heave_;          //!< Open loop force model for the heave DoF.
  Poly poly_yaw_;            //!< Open loop torque model for the yaw DoF.

  // Thruster allocator
  std::shared_ptr<ThrusterAllocatorWrapper> thruster_allocator_ptr_;  //!< Thruster allocator wrapper.

  // Enable and disable thrusters
  bool thrusters_enabled_;

  /**
   * @brief This is the callback for the navigation message.
   *
   * @param msg Navigation message.
   */
  void navCallback(const cola2_msgs::NavSts& msg);

  /**
   * @brief This is the callback for the ACK safety message.
   *
   * @param msg ACK safety message.
   */
  void ackCallback(const std_msgs::String& msg);

  /**
   * @brief Slow timer callback.
   *
   * @param event Timer event.
   */
  void timerSlowCallback(const ros::TimerEvent& event);

  /**
   * @brief Fast timer callback.
   *
   * @param event Timer event.
   */
  void timerFastCallback(const ros::TimerEvent& event);

  /**
   * @brief This is the main callback. Data is received, processed and sent to pose and velocity controllers.
   *
   * @param data Input Joy message.
   */
  void outputCallback(const sensor_msgs::Joy& data);

  /**
   * @brief Change max/min joy velocity.
   *
   * @param req Service request.
   * @param res Service response.
   * @return Service success.
   */
  bool setMaxJoyVelCallback(cola2_msgs::MaxJoyVelocity::Request& req, cola2_msgs::MaxJoyVelocity::Response& res);

  /**
   * @brief Set all joystick axes to velocity control.
   *
   * @param req Service request.
   * @param res Service response.
   * @return Service success.
   */
  bool setJoyAxesToVelCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Enable thrusters service callback.
   *
   * @param req Service request.
   * @param res Service response.
   * @return Service success.
   */
  bool enableThrustersCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Disable thrusters service callback.
   *
   * @param req Service request.
   * @param res Service response.
   * @return Service success.
   */
  bool disableThrustersCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Reload joystick config values.
   *
   * @param req Service request.
   * @param res Service response.
   * @return Service success.
   */
  bool reloadParamsCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Get config from ROS param server.
   *
   * @return Returns true if the config is successfully loaded, and false otherwise.
   */
  bool getConfig();

  /**
   * @brief This method can be used to handle a Trigger service call.
   *
   * @param srv_name Service name.
   * @param timeout Timeout.
   * @param wait_time Wait time.
   * @return Returns true if the call is successful and false otherwise.
   */
  bool callTriggerService(const std::string& srv_name, const double timeout, const double wait_time);

public:
  /**
   * @brief Default constructor.
   */
  Teleoperation();
};

Teleoperation::Teleoperation()
  : nh_("~")
  , ns_(cola2::ros::getNamespace())
  , diagnostic_(nh_, "teleoperation", cola2::ros::getUnresolvedNodeName())
  , last_map_ack_(0.0)
  , map_ack_init_(false)
  , map_ack_alive_(true)
  , seq_(0)
  , last_navigation_(0.0)
  , last_pose_(6, 0.0)
  , has_navigation_(false)
  , max_vel_(6, 0.0)
  , min_vel_(6, 0.0)
  , max_pos_(6, 0.0)
  , min_pos_(6, 0.0)
  , base_pose_(6, 0.0)
  , actualize_base_pose_(true)
  , pose_controlled_axis_(6, false)
  , last_nonzero_command_time_(0.0)
  , reset_keep_position_called_(true)
  , max_wrench_surge_(0.0)
  , max_wrench_sway_(0.0)
  , max_wrench_heave_(0.0)
  , max_wrench_yaw_(0.0)
  , poly_surge_("poly_surge")
  , poly_sway_("poly_sway")
  , poly_heave_("poly_heave")
  , poly_yaw_("poly_yaw")
  , thrusters_enabled_(false)
{
  // Wait for time
  while ((ros::Time::now().toSec() == 0.0) && (!ros::isShuttingDown()))
  {
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  // Get config. It must be valid, at least the first time
  if (!getConfig())
  {
    ROS_FATAL("Invalid parameters in ROS param server. Shutting down");
    ros::shutdown();
  }

  // Publishers
  pub_bvr_ = nh_.advertise<cola2_msgs::BodyVelocityReq>(ns_ + "/controller/body_velocity_req", 10);
  pub_wwr_ = nh_.advertise<cola2_msgs::WorldWaypointReq>(ns_ + "/controller/world_waypoint_req", 10);
  pub_check_joystick_ = nh_.advertise<std_msgs::String>("ack", 10);
  pub_thruster_setpoints_ = nh_.advertise<cola2_msgs::Setpoints>(ns_ + "/controller/thruster_setpoints", 1);

  // Subscribers
  sub_ack_ = nh_.subscribe(ns_ + "/input_to_teleoperation/ack", 10, &Teleoperation::ackCallback, this);
  sub_output_ = nh_.subscribe(ns_ + "/input_to_teleoperation/output", 10, &Teleoperation::outputCallback, this);
  sub_nav_ = nh_.subscribe(ns_ + "/navigator/navigation", 10, &Teleoperation::navCallback, this);

  // Services
  srv_set_max_joy_vel_ = nh_.advertiseService("set_max_joy_velocity", &Teleoperation::setMaxJoyVelCallback, this);
  srv_set_joy_axes_to_vel_ =
      nh_.advertiseService("set_joystick_axes_to_velocity", &Teleoperation::setJoyAxesToVelCallback, this);
  srv_reload_params_ = nh_.advertiseService("reload_params", &Teleoperation::reloadParamsCallback, this);
  srv_enable_thrusters_ = nh_.advertiseService("enable_thrusters", &Teleoperation::enableThrustersCallback, this);
  srv_disable_thrusters_ = nh_.advertiseService("disable_thrusters", &Teleoperation::disableThrustersCallback, this);

  // Timers
  timer_slow_ = nh_.createTimer(ros::Duration(1.0), &Teleoperation::timerSlowCallback, this);
  timer_fast_ = nh_.createTimer(ros::Duration(0.1), &Teleoperation::timerFastCallback, this);

  // Show message
  diagnostic_.setEnabled(true);
  ROS_INFO("Initialized");
}

void Teleoperation::navCallback(const cola2_msgs::NavSts& msg)
{
  if (cola2::ros::navigationIsValid(msg))
  {
    last_navigation_ = msg.header.stamp.toSec();
    last_pose_[0] = msg.position.north;
    last_pose_[1] = msg.position.east;
    last_pose_[2] = msg.position.depth;
    last_pose_[3] = msg.orientation.roll;
    last_pose_[4] = msg.orientation.pitch;
    last_pose_[5] = msg.orientation.yaw;
    has_navigation_ = true;
  }
}

void Teleoperation::ackCallback(const std_msgs::String& msg)
{
  // Very rudimentary parsing here...
  if (msg.data.size() >= 5)
  {
    const std::vector<std::string> data = { msg.data.substr(0, msg.data.size() - 4),
                                            msg.data.substr(msg.data.size() - 3) };
    if ((data[1] == "ack") && (data[0] == std::to_string(seq_ + 1)))
    {
      map_ack_alive_ = true;
      if (!map_ack_init_)
      {
        ROS_INFO("Teleoperation link established");
      }
      map_ack_init_ = true;
      ++seq_;
      last_map_ack_ = ros::Time::now().toSec();
    }
  }
}

void Teleoperation::timerSlowCallback(const ros::TimerEvent& event)
{
  diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);

  if (map_ack_init_)
  {
    // If there is a mission running, update last_map_ack so last_ack will be 0 and will start counting again once
    // the mission finishes
    diagnostic_.addKeyValue("last_ack", event.current_real.toSec() - last_map_ack_);
    if (map_ack_alive_)
    {
      map_ack_alive_ = false;
      diagnostic_.reportValidData(event.current_real);
    }
    else
    {
      // ROS_INFO("Missing teleoperation link");
      diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Missing teleoperation link");
      diagnostic_.reportData();

      /*cola2_msgs::BodyVelocityReq body_velocity_req;
      body_velocity_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_LOW;
      body_velocity_req.goal.requester = cola2::ros::getUnresolvedNodeName() + "_vel";
      body_velocity_req.twist.linear.x = 0.0;
      body_velocity_req.twist.linear.y = 0.0;
      body_velocity_req.twist.linear.z = 0.0;
      body_velocity_req.twist.angular.x = 0.0;
      body_velocity_req.twist.angular.y = 0.0;
      body_velocity_req.twist.angular.z = 0.0;
      body_velocity_req.disable_axis.x = true;
      body_velocity_req.disable_axis.y = true;
      body_velocity_req.disable_axis.z = true;
      body_velocity_req.disable_axis.roll = true;
      body_velocity_req.disable_axis.pitch = true;
      body_velocity_req.disable_axis.yaw = true;
      body_velocity_req.header.stamp = event.current_real;
      body_velocity_req.header.frame_id = cola2::ros::getNamespaceNoInitialDash() + "/base_link";
      pub_bvr_.publish(body_velocity_req);

      cola2_msgs::WorldWaypointReq world_waypoint_req;
      world_waypoint_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_LOW;
      world_waypoint_req.goal.requester = cola2::ros::getUnresolvedNodeName() + "_pose";
      world_waypoint_req.disable_axis.x = true;
      world_waypoint_req.disable_axis.y = true;
      world_waypoint_req.disable_axis.z = true;
      world_waypoint_req.disable_axis.roll = true;
      world_waypoint_req.disable_axis.pitch = true;
      world_waypoint_req.disable_axis.yaw = true;
      world_waypoint_req.header.stamp = event.current_real;
      world_waypoint_req.header.frame_id = "world_ned";
      pub_wwr_.publish(world_waypoint_req);*/

      // Reset pose controlled axis
      pose_controlled_axis_ = std::vector<bool>(6, false);
    }
  }
  else
  {
    ROS_INFO_THROTTLE(5.0, "Teleoperation link not established yet");
  }

  // Publish diagnostics
  diagnostic_.publish(event.current_real);

  // Send ack message. The other end needs to reply with the corresponding sequence number
  std_msgs::String msg;
  msg.data = std::to_string(seq_) + " ok";
  pub_check_joystick_.publish(msg);
}

void Teleoperation::timerFastCallback(const ros::TimerEvent& event)
{
  // Check whether navigation expired or not
  if (event.current_real.toSec() - last_navigation_ > 1.0)
  {
    has_navigation_ = false;
  }

  // Call reset keep position if needed
  if ((!reset_keep_position_called_) && (event.current_real.toSec() - last_nonzero_command_time_ > 3.0))
  {
    callTriggerService(ns_ + "/captain/reset_keep_position", 1.0, 1.0);
    reset_keep_position_called_ = true;
  }
}

void Teleoperation::outputCallback(const sensor_msgs::Joy& data)
{
  // Check axes and buttons length
  if (data.axes.size() != 12)
  {
    ROS_ERROR("Invalid axes array length");
    return;
  }
  if (data.buttons.size() != 12)
  {
    ROS_ERROR("Invalid buttons array length");
    return;
  }

  // Store time if the command is different than zero. This time is used by the reset keep position feature
  if (data.header.frame_id != "captain")
  {
    for (std::size_t i = 0; i < data.axes.size(); ++i)
    {
      if (std::fabs(data.axes[i]) > 1e-3)
      {
        last_nonzero_command_time_ = ros::Time::now().toSec();
        reset_keep_position_called_ = false;
      }
    }
    for (std::size_t i = 0; i < data.buttons.size(); ++i)
    {
      if (std::fabs(data.buttons[i]) > 1e-3)
      {
        last_nonzero_command_time_ = ros::Time::now().toSec();
        reset_keep_position_called_ = false;
      }
    }
  }

  // Check if pose controller is enabled
  for (std::size_t i = 0; i < 6; ++i)
  {
    if (data.buttons[i] == 1)
    {
      pose_controlled_axis_[i] = true;
      if (actualize_base_pose_)
      {
        base_pose_[i] = last_pose_[i];
      }
      ROS_INFO_STREAM("Axis " << i << " now is pose");
    }
  }

  // Check if velocity controller is enabled
  for (std::size_t i = 6; i < 12; ++i)
  {
    if (data.buttons[i] == 1)
    {
      pose_controlled_axis_[i - 6] = false;
      ROS_INFO_STREAM("Axis " << i - 6 << " now is velocity");
    }
  }

  // With navigation, we use position and velocity requests. Without navigation, we use open loop thruster commands
  if (has_navigation_)
  {
    // Compute desired positions
    std::vector<double> desired(12, 0);
    for (std::size_t i = 0; i < 6; ++i)
    {
      if (data.axes[i] < 0.0)
      {
        desired[i] = std::fabs(data.axes[i]) * min_pos_[i] + base_pose_[i];
      }
      else
      {
        desired[i] = data.axes[i] * max_pos_[i] + base_pose_[i];
      }
      if (i > 2)
      {
        // Normalize angles
        desired[i] = cola2::utils::wrapAngle(desired[i]);
      }
    }

    // Compute desired velocities
    for (std::size_t i = 6; i < 12; ++i)
    {
      if (data.axes[i] < 0.0)
      {
        desired[i] = std::fabs(data.axes[i]) * min_vel_[i - 6];
      }
      else
      {
        desired[i] = data.axes[i] * max_vel_[i - 6];
      }
    }

    // Publish world waypoint request
    cola2_msgs::WorldWaypointReq world_waypoint_req;
    world_waypoint_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION;
    world_waypoint_req.goal.requester = cola2::ros::getUnresolvedNodeName() + "_pose";
    world_waypoint_req.position.north = desired[0];
    world_waypoint_req.position.east = desired[1];
    world_waypoint_req.position.depth = desired[2];
    world_waypoint_req.orientation.roll = desired[3];
    world_waypoint_req.orientation.pitch = desired[4];
    world_waypoint_req.orientation.yaw = desired[5];
    world_waypoint_req.disable_axis.x = !pose_controlled_axis_[0];
    world_waypoint_req.disable_axis.y = !pose_controlled_axis_[1];
    world_waypoint_req.disable_axis.z = !pose_controlled_axis_[2];
    world_waypoint_req.disable_axis.roll = !pose_controlled_axis_[3];
    world_waypoint_req.disable_axis.pitch = !pose_controlled_axis_[4];
    world_waypoint_req.disable_axis.yaw = !pose_controlled_axis_[5];
    world_waypoint_req.header.stamp = ros::Time::now();
    world_waypoint_req.header.frame_id = "world_ned";
    if (world_waypoint_req.disable_axis.x && world_waypoint_req.disable_axis.y && world_waypoint_req.disable_axis.z &&
        world_waypoint_req.disable_axis.roll && world_waypoint_req.disable_axis.pitch &&
        world_waypoint_req.disable_axis.yaw)
    {
      world_waypoint_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_LOW;
    }
    pub_wwr_.publish(world_waypoint_req);

    // Create body velocity request
    cola2_msgs::BodyVelocityReq body_velocity_req;
    body_velocity_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION;
    body_velocity_req.goal.requester = cola2::ros::getUnresolvedNodeName() + "_vel";
    body_velocity_req.twist.linear.x = desired[6];
    body_velocity_req.twist.linear.y = desired[7];
    body_velocity_req.twist.linear.z = desired[8];
    body_velocity_req.twist.angular.x = desired[9];
    body_velocity_req.twist.angular.y = desired[10];
    body_velocity_req.twist.angular.z = desired[11];
    body_velocity_req.disable_axis.x = pose_controlled_axis_[0];
    body_velocity_req.disable_axis.y = pose_controlled_axis_[1];
    body_velocity_req.disable_axis.z = pose_controlled_axis_[2];
    body_velocity_req.disable_axis.roll = pose_controlled_axis_[3];
    body_velocity_req.disable_axis.pitch = pose_controlled_axis_[4];
    body_velocity_req.disable_axis.yaw = pose_controlled_axis_[5];

    // Check if DoF need to be disabled due to very small command
    if (std::fabs(body_velocity_req.twist.linear.x) < 0.025)
    {
      body_velocity_req.disable_axis.x = true;
    }
    if (std::fabs(body_velocity_req.twist.linear.y) < 0.025)
    {
      body_velocity_req.disable_axis.y = true;
    }
    if (std::fabs(body_velocity_req.twist.linear.z) < 0.025)
    {
      body_velocity_req.disable_axis.z = true;
    }
    if (std::fabs(body_velocity_req.twist.angular.x) < 0.01)
    {
      body_velocity_req.disable_axis.roll = true;
    }
    if (std::fabs(body_velocity_req.twist.angular.y) < 0.01)
    {
      body_velocity_req.disable_axis.pitch = true;
    }
    if (std::fabs(body_velocity_req.twist.angular.z) < 0.01)
    {
      body_velocity_req.disable_axis.yaw = true;
    }

    // If all DoF are disabled set priority to low
    if (body_velocity_req.disable_axis.x && body_velocity_req.disable_axis.y && body_velocity_req.disable_axis.z &&
        body_velocity_req.disable_axis.roll && body_velocity_req.disable_axis.pitch &&
        body_velocity_req.disable_axis.yaw)
    {
      body_velocity_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_TELEOPERATION_LOW;
    }

    // Publish body velocity request
    body_velocity_req.header.stamp = ros::Time::now();
    body_velocity_req.header.frame_id = cola2::ros::getNamespaceNoInitialDash() + "/base_link";
    pub_bvr_.publish(body_velocity_req);
  }
  else
  {
    // Display message
    ROS_INFO_THROTTLE(5.0, "Teleoperation working in open loop mode");

    if (thrusters_enabled_)
    {
      // Compute desired velocities
      const double desired_surge_vel = data.axes[6] * OPEN_LOOP_MAX_SURGE_VEL;
      const double desired_sway_vel = data.axes[7] * OPEN_LOOP_MAX_SWAY_VEL;
      const double desired_heave_vel = data.axes[8] * OPEN_LOOP_MAX_HEAVE_VEL;
      const double desired_yaw_rate = data.axes[11] * OPEN_LOOP_MAX_YAW_RATE;

      // Compute desired forces using the open loop poly models
      const double desired_surge_force = poly_surge_.compute(0.0, desired_surge_vel, 0.0);
      const double desired_sway_force = poly_sway_.compute(0.0, desired_sway_vel, 0.0);
      const double desired_heave_force = poly_heave_.compute(0.0, desired_heave_vel, 0.0);
      const double desired_yaw_torque = poly_yaw_.compute(0.0, desired_yaw_rate, 0.0);

      // Compute thruster forces
      const std::vector<double> wrench = { desired_surge_force, desired_sway_force, desired_heave_force, 0.0, 0.0,
                                           desired_yaw_torque };
      const std::vector<double> thruster_forces = thruster_allocator_ptr_->wrenchToThrusterForcesAsVector(wrench);

      // Compute thruster setpoints
      const std::vector<double> setpoints = thruster_allocator_ptr_->forcesToSetpointsAsVector(thruster_forces);

      // Publish setpoints
      cola2_msgs::Setpoints msg_setpoints;
      msg_setpoints.header.stamp = ros::Time::now();  // Safer than using joy stamp
      msg_setpoints.header.frame_id = "open_loop";    // The frame_id must be "open_loop"
      msg_setpoints.setpoints = setpoints;
      pub_thruster_setpoints_.publish(msg_setpoints);
    }
    else
    {
      ROS_INFO_THROTTLE(5.0, "Discarding open loop mode setpoints because thrusters are not enabled");
    }
  }
}

bool Teleoperation::setMaxJoyVelCallback(cola2_msgs::MaxJoyVelocity::Request& req,
                                         cola2_msgs::MaxJoyVelocity::Response& res)
{
  ROS_INFO("Change max/min joy velocity");
  for (std::size_t i = 0; i < 6; ++i)
  {
    max_vel_[i] = req.max_joy_velocity[i];
    min_vel_[i] = -req.max_joy_velocity[i];
  }
  res.attempted = true;
  return true;
}

bool Teleoperation::setJoyAxesToVelCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  // Set all axis at 0.0 and set control to velocity for all axes
  ROS_INFO("Set all axis to velocity");
  sensor_msgs::Joy data;
  data.axes.resize(12, 0.0);
  data.buttons.resize(6, 0);
  data.buttons.resize(12, 1);
  outputCallback(data);
  res.message = "Success";
  res.success = true;
  return true;
}

bool Teleoperation::enableThrustersCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Enabling thrusters");
  if (callTriggerService(ns_ + "/controller/enable_thrusters", 1.0, 1.0))
  {
    thrusters_enabled_ = true;
    res.message = "Thrusters enabled";
    res.success = true;
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    res.message = "Error enabling thrusters. Controller service call failed";
    res.success = false;
    ROS_ERROR_STREAM(res.message);
  }
  return true;
}

bool Teleoperation::disableThrustersCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Disabling thrusters");
  if (callTriggerService(ns_ + "/controller/disable_thrusters", 1.0, 1.0))
  {
    callTriggerService(ns_ + "/safety_supervisor/reset_emergency_ramp", 1.0, 1.0);
    thrusters_enabled_ = false;
    res.message = "Thrusters disabled";
    res.success = true;
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    res.message = "Error disabling thrusters. Controller service call failed";
    res.success = false;
    ROS_ERROR_STREAM(res.message);
  }
  return true;
}

bool Teleoperation::reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Reload teleoperation params");
  if (getConfig())
  {
    res.message = "Success";
    res.success = true;
    callTriggerService(ns_ + "/param_logger/publish_params", 1.0, 1.0);
  }
  else
  {
    res.message = "Unable to successfully reload params";
    res.success = false;
    ROS_WARN_STREAM(res.message);
  }

  return true;
}

bool Teleoperation::getConfig()
{
  bool ok = true;

  // Normal teleoperation parameters
  std::vector<double> max_pos;
  std::vector<double> min_pos;
  std::vector<double> max_vel;
  std::vector<double> min_vel;
  std::vector<bool> pose_controlled_axis;
  std::vector<double> base_pose;
  bool actualize_base_pose;
  ok &= cola2::ros::getParam("~max_pos", max_pos);
  ok &= cola2::ros::getParam("~min_pos", min_pos);
  ok &= cola2::ros::getParam("~max_vel", max_vel);
  ok &= cola2::ros::getParam("~min_vel", min_vel);
  ok &= cola2::ros::getParam("~pose_controlled_axis", pose_controlled_axis);
  ok &= cola2::ros::getParam("~base_pose", base_pose);
  ok &= cola2::ros::getParam("~actualize_base_pose", actualize_base_pose);

  // Open loop teleoperation parameters
  int n_thrusters = 0;
  ok &= cola2::ros::getParam(ns_ + "/controller/n_thrusters", n_thrusters);
  std::vector<std::vector<double> > poly_positive_v, poly_negative_v;
  std::vector<double> max_force_thruster_positive_v, max_force_thruster_negative_v;
  for (int i = 0; i < n_thrusters; ++i)
  {
    std::vector<double> thruster_poly_positive, thruster_poly_negative;
    ok &= cola2::ros::getParamVector(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_poly_positive",
                                     thruster_poly_positive);
    ok &= cola2::ros::getParamVector(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_poly_negative",
                                     thruster_poly_negative);
    poly_positive_v.push_back(thruster_poly_positive);
    poly_negative_v.push_back(thruster_poly_negative);

    double max_force_positive, max_force_negative;
    ok &= cola2::ros::getParam(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_max_force_positive",
                               max_force_positive);
    ok &= cola2::ros::getParam(ns_ + "/controller/thruster_" + std::to_string(i + 1) + "_max_force_negative",
                               max_force_negative);
    max_force_thruster_positive_v.push_back(max_force_positive);
    max_force_thruster_negative_v.push_back(max_force_negative);
  }
  std::vector<double> tcm;
  ok &= cola2::ros::getParamVector(ns_ + "/controller/TCM", tcm);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_X", max_wrench_surge_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Y", max_wrench_sway_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Z", max_wrench_heave_);
  ok &= cola2::ros::getParam(ns_ + "/controller/max_wrench_Yaw", max_wrench_yaw_);
  double poly_surge_a, poly_surge_b, poly_surge_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_A", poly_surge_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_B", poly_surge_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_surge_C", poly_surge_c);
  std::map<std::string, double> poly_surge_params;
  poly_surge_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_surge_params.insert(std::pair<std::string, double>("0", poly_surge_a));
  poly_surge_params.insert(std::pair<std::string, double>("1", poly_surge_b));
  poly_surge_params.insert(std::pair<std::string, double>("2", poly_surge_c));
  double poly_sway_a, poly_sway_b, poly_sway_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_A", poly_sway_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_B", poly_sway_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_sway_C", poly_sway_c);
  std::map<std::string, double> poly_sway_params;
  poly_sway_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_sway_params.insert(std::pair<std::string, double>("0", poly_sway_a));
  poly_sway_params.insert(std::pair<std::string, double>("1", poly_sway_b));
  poly_sway_params.insert(std::pair<std::string, double>("2", poly_sway_c));
  double poly_heave_a, poly_heave_b, poly_heave_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_A", poly_heave_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_B", poly_heave_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_heave_C", poly_heave_c);
  std::map<std::string, double> poly_heave_params;
  poly_heave_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_heave_params.insert(std::pair<std::string, double>("0", poly_heave_a));
  poly_heave_params.insert(std::pair<std::string, double>("1", poly_heave_b));
  poly_heave_params.insert(std::pair<std::string, double>("2", poly_heave_c));
  double poly_yaw_a, poly_yaw_b, poly_yaw_c;
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_A", poly_yaw_a);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_B", poly_yaw_b);
  ok &= cola2::ros::getParam(ns_ + "/controller/poly_yaw_C", poly_yaw_c);
  std::map<std::string, double> poly_yaw_params;
  poly_yaw_params.insert(std::pair<std::string, double>("n_dof", 3.0));
  poly_yaw_params.insert(std::pair<std::string, double>("0", poly_yaw_a));
  poly_yaw_params.insert(std::pair<std::string, double>("1", poly_yaw_b));
  poly_yaw_params.insert(std::pair<std::string, double>("2", poly_yaw_c));

  if (ok)
  {
    // Copy normal teleoperation parameters
    max_pos_ = max_pos;
    min_pos_ = min_pos;
    max_vel_ = max_vel;
    min_vel_ = min_vel;
    pose_controlled_axis_ = pose_controlled_axis;
    base_pose_ = base_pose;
    actualize_base_pose_ = actualize_base_pose;

    // Copy open loop teleoperation parameters
    poly_surge_.setParameters(poly_surge_params);
    poly_sway_.setParameters(poly_sway_params);
    poly_heave_.setParameters(poly_heave_params);
    poly_yaw_.setParameters(poly_yaw_params);
    thruster_allocator_ptr_ = std::make_shared<ThrusterAllocatorWrapper>(n_thrusters);
    thruster_allocator_ptr_->setParams(max_force_thruster_positive_v, max_force_thruster_negative_v, poly_positive_v,
                                       poly_negative_v, tcm);
  }

  return ok;
}

bool Teleoperation::callTriggerService(const std::string& srv_name, const double timeout, const double wait_time)
{
  try
  {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    const bool success =
        cola2::ros::callServiceWithTimeout<std_srvs::Trigger>(nh_, req, res, srv_name, timeout, wait_time);
    if (!success)
    {
      ROS_ERROR_STREAM("Trigger service " << srv_name << " call failed");
    }
    else
    {
      if (!res.success)
      {
        ROS_WARN_STREAM("Trigger service " << srv_name << " responded False with msg: " << res.message);
      }
      else
        return true;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Exception while calling trigger service " << srv_name << ": " << ex.what());
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleoperation");
  Teleoperation teleoperation;
  ros::spin();
  return 0;
}
