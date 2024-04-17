/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/virtual_cage.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
  VirtualCage::VirtualCage(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , enables_abort_and_surface_(false)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "enabled", "enabled", DataType::Bool},
    {std::string("/safety/") + rule_name_, "inside_virtual_cage", "inside_virtual_cage", DataType::Bool}
  });
  setParseList(parse_list);
}

void
VirtualCage::parseDiagnostics()
{
  if (hasBool("enabled"))
    last_valid_diagnostics_data_ = last_diagnostic_;
}

void
VirtualCage::periodicUpdate(const ros::Time& stamp, std::uint32_t*)
{
  // Clear level and message
  level_ = SafetyLevel::NONE;
  message_.clear();

  // Check data
  if (last_valid_config_ == 0.0)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("invalid config");
  }
  else if (last_valid_diagnostics_data_ == 0.0)
  {
    message_ = createMessage("waiting for valid diagnostics");
    if (stamp.toSec() - last_valid_config_ < INIT_TIME)
      level_ = SafetyLevel::INFORMATIVE;
    else
      level_ = SafetyLevel::ABORT_AND_SURFACE;
  }
  else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_ESCALATED_TIME)
  {
    level_ = SafetyLevel::EMERGENCY_SURFACE;
    message_ = createMessage("too much time without valid diagnostics, escalated to emergency");
  }
  else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_TIME)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("too much time without valid diagnostics");
  }
  else if (hasBool("inside_virtual_cage") && (!getBool("inside_virtual_cage")))
  {
    message_ = createMessage("outside virtual cage");
    if (enables_abort_and_surface_)
      level_ = SafetyLevel::ABORT_AND_SURFACE;
    else
      level_ = SafetyLevel::INFORMATIVE;
  }
}

bool
VirtualCage::loadConfigFromParamServer()
{
  // Load config from param server
  bool temp_enables_abort_and_surface;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/enables_abort_and_surface"), temp_enables_abort_and_surface);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  enables_abort_and_surface_ = temp_enables_abort_and_surface;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
