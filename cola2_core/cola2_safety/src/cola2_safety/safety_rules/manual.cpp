/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/manual.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/RecoveryAction.h>

namespace SafetyRules
{
Manual::Manual(const std::string& rule_name, ros::NodeHandle* nh_ptr)
  : SafetyRuleBaseClass(rule_name)
{
  loadConfigFromParamServer();

  // Service
  srv_ = nh_ptr->advertiseService(rule_name + std::string("/set_level"), &Manual::serviceCallback, this);
}

bool
Manual::serviceCallback(cola2_msgs::Recovery::Request& req, cola2_msgs::Recovery::Response& res)
{
  message_ = createMessage(req.requested_action.error_string);
  if (req.requested_action.error_level == cola2_msgs::RecoveryAction::NONE)
  {
    level_ = SafetyLevel::NONE;
    message_.clear();
  }
  else if (req.requested_action.error_level == cola2_msgs::RecoveryAction::INFORMATIVE)
    level_ = SafetyLevel::INFORMATIVE;
  else if (req.requested_action.error_level == cola2_msgs::RecoveryAction::ABORT)
    level_ = SafetyLevel::ABORT;
  else if (req.requested_action.error_level == cola2_msgs::RecoveryAction::ABORT_AND_SURFACE)
    level_ = SafetyLevel::ABORT_AND_SURFACE;
  else if (req.requested_action.error_level == cola2_msgs::RecoveryAction::EMERGENCY_SURFACE)
    level_ = SafetyLevel::EMERGENCY_SURFACE;
  else if (req.requested_action.error_level == cola2_msgs::RecoveryAction::DROP_WEIGHT)
    level_ = SafetyLevel::DROP_WEIGHT;
  else
  {
    level_ = SafetyLevel::NONE;
    ROS_ERROR_STREAM(createMessage("unknown safety level"));
  }
  res.attempted = true;
  return true;
}
}  // namespace SafetyRules
