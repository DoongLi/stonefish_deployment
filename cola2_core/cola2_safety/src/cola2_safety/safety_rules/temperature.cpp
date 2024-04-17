/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/temperature.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
  Temperature::Temperature(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , temperature_(0.0)
  , max_temperature_(0.0)
{
  loadConfigFromParamServer();
}

void
Temperature::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
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
  else if (temperature_ > max_temperature_)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("temperature above threshold");
  }

  if ((level_ != SafetyLevel::NONE) && (level_ != SafetyLevel::INFORMATIVE))
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::HIGH_TEMPERATURE, true);
}

bool
Temperature::loadConfigFromParamServer()
{
  // Load config from param server
  std::string temp_diagnostic_status_name;
  double temp_max_temperature;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/diagnostic_status_name"), temp_diagnostic_status_name);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/max_temperature"), temp_max_temperature);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  diagnostic_status_name_ = temp_diagnostic_status_name;
  max_temperature_ = temp_max_temperature;
  last_valid_config_ = ros::Time::now().toSec();
  updateParseList();
  return ok;
}

void
Temperature::updateParseList()
{
  const ParseList parse_list({
    {diagnostic_status_name_, "temperature", "temperature", DataType::Double}
  });
  setParseList(parse_list);
}

void
Temperature::parseDiagnostics()
{
  if (hasDouble("temperature"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    temperature_ = getDouble("temperature");
  }
}
}  // namespace SafetyRules
