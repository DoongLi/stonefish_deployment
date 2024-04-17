/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/battery_level.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
BatteryLevel::BatteryLevel(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , charge_(0.0)
  , voltage_(0.0)
  , min_charge_(0.0)
  , min_voltage_(0.0)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "charge", "charge", DataType::Double},
    {std::string("/safety/") + rule_name_, "voltage", "voltage", DataType::Double}
  });
  setParseList(parse_list);
}

void
BatteryLevel::parseDiagnostics()
{
  if (hasDouble("charge") && hasDouble("voltage"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    charge_ = getDouble("charge");
    voltage_ = getDouble("voltage");
  }
}

void
BatteryLevel::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
{
  // Clear level, message and status code bits
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
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_ERROR, true);
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_WARNING, true);
  }
  else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_TIME)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("too much time without valid diagnostics");
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_ERROR, true);
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_WARNING, true);
  }
  else if ((charge_ < min_charge_) || (voltage_ < min_voltage_))
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    if ((charge_ < min_charge_) && (voltage_ < min_voltage_))
      message_ = createMessage("battery charge and voltage below threshold");
    else if (charge_ < min_charge_)
      message_ = createMessage("battery charge below threshold");
    else
      message_ = createMessage("battery voltage below threshold");
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_ERROR, true);
  }
  else if ((charge_ < LOW_CHARGE_FACTOR * min_charge_) || (voltage_ < min_voltage_ + LOW_VOLTAGE_OFFSET))
  {
    level_ = SafetyLevel::INFORMATIVE;
    if ((charge_ < LOW_CHARGE_FACTOR * min_charge_) && (voltage_ < min_voltage_ + LOW_VOLTAGE_OFFSET))
      message_ = createMessage("low battery charge and voltage");
    else if (charge_ < LOW_CHARGE_FACTOR * min_charge_)
      message_ = createMessage("low battery charge");
    else
      message_ = createMessage("low battery voltage");
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::LOW_BATTERY_WARNING, true);
  }
}

bool
BatteryLevel::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_min_charge;
  double temp_min_voltage;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/min_charge"), temp_min_charge);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/min_voltage"), temp_min_voltage);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  min_charge_ = temp_min_charge;
  min_voltage_ = temp_min_voltage;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
