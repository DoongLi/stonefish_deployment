/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/serviceclient_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/Mission.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <cola2_msgs/Setpoints.h>
#include <cola2_safety/safety_rules/battery_control_board.h>
#include <cola2_safety/safety_rules/battery_level.h>
#include <cola2_safety/safety_rules/captain.h>
#include <cola2_safety/safety_rules/check_enabled.h>
#include <cola2_safety/safety_rules/combined_water_inside.h>
#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/comms.h>
#include <cola2_safety/safety_rules/manual.h>
#include <cola2_safety/safety_rules/navigator.h>
#include <cola2_safety/safety_rules/teleoperation.h>
#include <cola2_safety/safety_rules/temperature.h>
#include <cola2_safety/safety_rules/virtual_cage.h>
#include <cola2_safety/safety_rules/watchdog_timer.h>
#include <cola2_safety/safety_rules/water_inside.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

template <typename T>
bool callServiceHelper(ros::NodeHandle* nh_ptr, const std::string& srv_name)
{
  const double time_init = ros::Time::now().toSec();
  bool valid_call = false;
  try
  {
    typename T::Request req;
    typename T::Response res;
    const bool success = cola2::ros::callServiceWithTimeout<T>(*nh_ptr, req, res, srv_name, 0.5);
    if (!success)
    {
      ROS_ERROR_STREAM("Service " << srv_name << " call failed");
    }
    else
    {
      if (!res.success)
      {
        ROS_WARN_STREAM("Service " << srv_name << " responded False with msg: " << res.message);
      }
    }
    valid_call = success && res.success;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("Exception while calling service " << srv_name << ": " << ex.what());
  }
  const double elapsed_time = ros::Time::now().toSec() - time_init;
  if (elapsed_time > 0.05)
  {
    ROS_WARN_STREAM("Slow service call of " << srv_name << " service (" << elapsed_time << " seconds)");
  }
  return valid_call;
}

class SafetySupervisor
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_sss_, pub_ra_, pub_thrusters_;
  ros::Subscriber sub_diag_, sub_captain_;
  ros::ServiceServer srv_reload_params_, srv_reset_emergency_ramp_;
  ros::Timer main_timer_;
  std::string ns_;
  std::vector<SafetyRules::SafetyRuleBaseClass*> rules_;
  std::size_t last_level_;
  std::string last_message_;
  bool in_safety_keep_position_;
  double last_captain_status_time_;
  double last_emergency_surface_time_;
  double last_message_rosout_time_;
  double init_time_;
  bool navigation_filter_init_;
  double navigation_filter_init_time_;

  // Config
  std::vector<double> emergency_surface_setpoints_;
  std::string drop_weight_service_;

  // Methods
  bool srvReloadParams(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool srvResetEmergencyRamp(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray&);
  void captainStatusCallback(const cola2_msgs::CaptainStatus&);
  void mainTimerCallback(const ros::TimerEvent&);

public:
  SafetySupervisor();
  ~SafetySupervisor();
};

SafetySupervisor::SafetySupervisor()
  : nh_("~")
  , ns_(cola2::ros::getNamespace())
  , last_level_(SafetyRules::SafetyLevel::NONE)
  , in_safety_keep_position_(false)
  , last_captain_status_time_(0.0)
  , last_emergency_surface_time_(0.0)
  , last_message_rosout_time_(0.0)
  , navigation_filter_init_(false)
{
  // Wait for time and initialize variables
  while (ros::Time::now().toSec() == 0.0)
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }
  init_time_ = ros::Time::now().toSec();
  navigation_filter_init_time_ = init_time_;

  // Load params from param server. These can only be loaded once
  bool ok = true;
  std::vector<std::string> safety_rule_names;
  ok &= cola2::ros::getParam("~emergency_surface_setpoints", emergency_surface_setpoints_);
  ok &= cola2::ros::getParam("~drop_weight_service", drop_weight_service_);
  ok &= cola2::ros::getParam("~safety_rule_names", safety_rule_names);
  if (!ok)
  {
    ROS_FATAL_STREAM("Wrong or missing basic configuration. Shutting down");
    ros::shutdown();
  }

  // Add safety rules according to the config
  for (const auto& rule_name : safety_rule_names)
  {
    std::string rule_type;
    if (!cola2::ros::getParam(std::string("~") + rule_name + std::string("/type"), rule_type))
    {
      ROS_ERROR_STREAM("Unable to load type for rule with name " << rule_name);
      continue;
    }

    if (rule_type.compare("BatteryControlBoard") == 0)
      rules_.push_back(new SafetyRules::BatteryControlBoard(rule_name));
    else if (rule_type.compare("BatteryLevel") == 0)
      rules_.push_back(new SafetyRules::BatteryLevel(rule_name));
    else if (rule_type.compare("Captain") == 0)
      rules_.push_back(new SafetyRules::Captain(rule_name, &nh_));
    else if (rule_type.compare("CheckEnabled") == 0)
      rules_.push_back(new SafetyRules::CheckEnabled(rule_name));
    else if (rule_type.compare("CombinedWaterInside") == 0)
      rules_.push_back(new SafetyRules::CombinedWaterInside(rule_name));
    else if (rule_type.compare("Comms") == 0)
      rules_.push_back(new SafetyRules::Comms(rule_name));
    else if (rule_type.compare("Teleoperation") == 0)
      rules_.push_back(new SafetyRules::Teleoperation(rule_name, &nh_));
    else if (rule_type.compare("Manual") == 0)
      rules_.push_back(new SafetyRules::Manual(rule_name, &nh_));
    else if (rule_type.compare("Navigator") == 0)
      rules_.push_back(new SafetyRules::Navigator(rule_name, &nh_));
    else if (rule_type.compare("Temperature") == 0)
      rules_.push_back(new SafetyRules::Temperature(rule_name));
    else if (rule_type.compare("VirtualCage") == 0)
      rules_.push_back(new SafetyRules::VirtualCage(rule_name));
    else if (rule_type.compare("WatchdogTimer") == 0)
      rules_.push_back(new SafetyRules::WatchdogTimer(rule_name));
    else if (rule_type.compare("WaterInside") == 0)
      rules_.push_back(new SafetyRules::WaterInside(rule_name));
    else
    {
      ROS_ERROR_STREAM("Invalid type " << rule_type << " for rule with name " << rule_name);
    }
  }

  // Publishers
  pub_sss_ = nh_.advertise<cola2_msgs::SafetySupervisorStatus>("status", 1, true);
  pub_ra_ = nh_.advertise<cola2_msgs::RecoveryAction>("rules_recovery_actions", 40);
  pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>(ns_ + "/controller/thruster_setpoints", 1, true);

  // Services
  srv_reload_params_ = nh_.advertiseService("reload_params", &SafetySupervisor::srvReloadParams, this);
  srv_reset_emergency_ramp_ =
      nh_.advertiseService("reset_emergency_ramp", &SafetySupervisor::srvResetEmergencyRamp, this);

  // Subscriber
  sub_diag_ = nh_.subscribe(ns_ + "/diagnostics_agg", 1, &SafetySupervisor::diagnosticsCallback, this);
  sub_captain_ = nh_.subscribe(ns_ + "/captain/captain_status", 1, &SafetySupervisor::captainStatusCallback, this);

  // Main timer
  main_timer_ = nh_.createTimer(ros::Duration(0.1), &SafetySupervisor::mainTimerCallback, this);

  ROS_INFO("Initialized");
}

SafetySupervisor::~SafetySupervisor()
{
  for (const auto& rule : rules_)
    delete rule;
}

bool SafetySupervisor::srvReloadParams(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Reload params service called");

  // Reload params in the rules
  bool ok = true;
  for (const auto& rule : rules_)
    ok &= rule->loadConfigFromParamServer();

  // Call publish params service regardless of the response from the rules
  callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/param_logger/publish_params");

  // Check if some rule failed
  if (ok)
  {
    res.message = "Success";
    res.success = true;
  }
  else
  {
    res.message = "Invalid parameters";
    res.success = false;
    ROS_ERROR_STREAM(res.message);
  }
  return true;
}

bool SafetySupervisor::srvResetEmergencyRamp(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Reset emergency ramp");
  last_emergency_surface_time_ = ros::Time::now().toSec();
  res.message = "Success";
  res.success = true;
  return true;
}

void SafetySupervisor::diagnosticsCallback(const diagnostic_msgs::DiagnosticArray& msg)
{
  // Pass the diagnostics to all the rules
  for (const auto& rule : rules_)
    rule->diagnosticsUpdate(msg);

  // Check filter init
  for (const auto& diagnostic_status : msg.status)
  {
    if (diagnostic_status.level == diagnostic_msgs::DiagnosticStatus::STALE)
      continue;
    if (diagnostic_status.name.compare("/navigation/navigator") != 0)
      continue;
    for (const auto& key_value : diagnostic_status.values)
    {
      if (key_value.key.compare("filter_init") == 0)
      {
        const bool navigation_filter_init = SafetyRules::SafetyRuleBaseClass::stringToBool(key_value.value);
        if (navigation_filter_init_ != navigation_filter_init)
          navigation_filter_init_time_ = ros::Time::now().toSec();
        navigation_filter_init_ = navigation_filter_init;
      }
    }
  }
}

void SafetySupervisor::captainStatusCallback(const cola2_msgs::CaptainStatus& msg)
{
  // This is just to avoid enabling the safety keep position over an over again
  in_safety_keep_position_ = (msg.state == cola2_msgs::CaptainStatus::SAFETYKEEPPOSITION);
  last_captain_status_time_ = ros::Time::now().toSec();
}

void SafetySupervisor::mainTimerCallback(const ros::TimerEvent& event)
{
  // Do periodic update, get max level and status code, and publish rule
  std::size_t level = 0;
  std::uint32_t status_code = 0;
  for (const auto& rule : rules_)
  {
    rule->periodicUpdate(event.current_real, &status_code);
    level = std::max(level, rule->getLevel());
    if (rule->getLevel() != SafetyRules::SafetyLevel::NONE)
    {
      cola2_msgs::RecoveryAction recovery_action;
      recovery_action.header.stamp = event.current_real;
      recovery_action.header.frame_id = rule->getRuleName();
      recovery_action.error_level = rule->getLevel();
      recovery_action.error_string = rule->getMessage();
      pub_ra_.publish(recovery_action);
    }
  }

  // Get message from the rules with max level
  std::string message;
  for (const auto& rule : rules_)
  {
    if (rule->getLevel() == level)
      message += (message.empty() ? rule->getMessage() : std::string(". ") + rule->getMessage());
  }

  // Modify safety level if the navigation filter is not initialized
  if (!(navigation_filter_init_ && (event.current_real.toSec() - navigation_filter_init_time_ > 5.0)))
  {
    if ((level == SafetyRules::SafetyLevel::ABORT) || (level == SafetyRules::SafetyLevel::ABORT_AND_SURFACE))
    {
      ROS_INFO_STREAM_THROTTLE(5.0, "Navigation filter is not initialized. Safety level decreased from "
                                        << SafetyRules::SafetyLevel::getName(level) << " to INFORMATIVE");
      level = SafetyRules::SafetyLevel::INFORMATIVE;
    }
  }

  // Display message
  if ((level != last_level_) || (message != last_message_) ||
      ((level != SafetyRules::SafetyLevel::NONE) && (event.current_real.toSec() - last_message_rosout_time_ > 5.0)))
  {
    if (level == SafetyRules::SafetyLevel::NONE)
    {
      ROS_INFO_STREAM("Safety level -> " << SafetyRules::SafetyLevel::getName(level));
    }
    else if (level == SafetyRules::SafetyLevel::INFORMATIVE)
    {
      if (event.current_real.toSec() - init_time_ > 5.0)
      {
        ROS_INFO_STREAM("Safety level -> " << SafetyRules::SafetyLevel::getName(level) << ". Message -> " << message);
      }
    }
    else
    {
      ROS_WARN_STREAM("Safety level -> " << SafetyRules::SafetyLevel::getName(level) << ". Message -> " << message);
    }
    last_message_rosout_time_ = event.current_real.toSec();
  }

  // Perform recovery action associated to safety level
  if (level == SafetyRules::SafetyLevel::ABORT)
  {
    SafetyRules::StatusCodeBits::setBit(&status_code, SafetyRules::StatusCodeBits::RA_ABORT, true);
    ROS_WARN_THROTTLE(1.0, "Recovery action: disabling mission, external mission and goto");
    if (last_level_ != level)
    {
      // This is done only once
      if (!callServiceHelper<cola2_msgs::Mission>(&nh_, ns_ + "/captain/disable_mission"))
      {
        ROS_ERROR("Error disabling mission");
      }
      if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/captain/disable_external_mission"))
      {
        ROS_ERROR("Error disabling external mission");
      }
      if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/captain/disable_goto"))
      {
        ROS_ERROR("Error disabling goto");
      }
    }
  }
  else if (level == SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
  {
    SafetyRules::StatusCodeBits::setBit(&status_code, SafetyRules::StatusCodeBits::RA_ABORT_SURFACE, true);
    ROS_WARN_THROTTLE(1.0, "Recovery action: enabling safety keep position");
    if ((last_captain_status_time_ == 0.0) || (event.current_real.toSec() - last_captain_status_time_ > 10.0) ||
        (!in_safety_keep_position_))
    {
      // This is done if not in safety keep position
      if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/captain/disable_all_and_set_idle"))
      {
        ROS_ERROR("Error disabling all and setting idle");
      }
      if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/captain/enable_safety_keep_position"))
      {
        ROS_ERROR("Error enabling safety keep position");
      }
    }
  }
  else if (level == SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
  {
    SafetyRules::StatusCodeBits::setBit(&status_code, SafetyRules::StatusCodeBits::RA_EMERGENCY_SURFACE, true);
    ROS_WARN_THROTTLE(1.0, "Recovery action: emergency surface");
    if (last_level_ != level)
    {
      // This is done only once
      last_emergency_surface_time_ = event.current_real.toSec();
      if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/captain/disable_all_and_set_idle"))
      {
        ROS_ERROR("Error disabling all and setting idle");
      }
    }

    // Send thruster setpoints (using ramp)
    cola2_msgs::Setpoints msg_setpoints;
    msg_setpoints.header.stamp = ros::Time::now();  // Safer than using event
    msg_setpoints.header.frame_id = "safety";
    const double factor =
        std::max(0.0, std::min(1.0, (event.current_real.toSec() - last_emergency_surface_time_) / 30.0));
    msg_setpoints.setpoints.reserve(emergency_surface_setpoints_.size());
    for (const auto& setpoint : emergency_surface_setpoints_)
      msg_setpoints.setpoints.push_back(factor * setpoint);
    pub_thrusters_.publish(msg_setpoints);
  }
  else if (level == SafetyRules::SafetyLevel::DROP_WEIGHT)
  {
    SafetyRules::StatusCodeBits::setBit(&status_code, SafetyRules::StatusCodeBits::RA_DROP_WEIGHT, true);
    ROS_WARN_THROTTLE(1.0, "Recovery action: drop weight");
    if (last_level_ != level)
    {
      // This is done only once
      if (drop_weight_service_.empty())
      {
        ROS_WARN("The drop weight service name is empty");
      }
      else if (!callServiceHelper<std_srvs::Trigger>(&nh_, ns_ + "/" + drop_weight_service_))
      {
        ROS_ERROR("Error calling drop weight service");
      }
    }

    // Send thruster setpoints (without ramp)
    cola2_msgs::Setpoints msg_setpoints;
    msg_setpoints.header.stamp = ros::Time::now();  // Safer than using event
    msg_setpoints.header.frame_id = "safety";
    msg_setpoints.setpoints.reserve(emergency_surface_setpoints_.size());
    for (const auto& setpoint : emergency_surface_setpoints_)
      msg_setpoints.setpoints.push_back(setpoint);
    pub_thrusters_.publish(msg_setpoints);
  }

  // Publish safety supervisor status
  cola2_msgs::SafetySupervisorStatus msg_sss;
  msg_sss.header.stamp = event.current_real;
  msg_sss.status_code = status_code;
  if (level == SafetyRules::SafetyLevel::NONE)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::NONE;
  else if (level == SafetyRules::SafetyLevel::INFORMATIVE)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::INFORMATIVE;
  else if (level == SafetyRules::SafetyLevel::ABORT)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::ABORT;
  else if (level == SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::ABORT_AND_SURFACE;
  else if (level == SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::EMERGENCY_SURFACE;
  else if (level == SafetyRules::SafetyLevel::DROP_WEIGHT)
    msg_sss.recovery_action.error_level = cola2_msgs::RecoveryAction::DROP_WEIGHT;
  msg_sss.recovery_action.error_string = message;
  msg_sss.recovery_action.header = msg_sss.header;
  pub_sss_.publish(msg_sss);

  // Update last level and last message
  last_level_ = level;
  last_message_ = message;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_supervisor");
  SafetySupervisor safety_supervisor;
  ros::spin();
  return 0;
}
