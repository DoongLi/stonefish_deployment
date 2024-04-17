/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/watchdog_timer.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
WatchdogTimer::WatchdogTimer(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , elapsed_time_(0.0)
  , timeout_(0.0)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "elapsed_time", "elapsed_time", DataType::Double}
  });
  setParseList(parse_list);
}

void
WatchdogTimer::parseDiagnostics()
{
  if (hasDouble("elapsed_time"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    elapsed_time_ = getDouble("elapsed_time");
  }
}

void
WatchdogTimer::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
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
  else if (stamp.toSec() - last_valid_diagnostics_data_ + elapsed_time_ > timeout_)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("timeout occurred");
  }

  if ((level_ != SafetyLevel::NONE) && (level_ != SafetyLevel::INFORMATIVE))
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::WATCHDOG_TIMER, true);
}

bool
WatchdogTimer::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_timeout;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/timeout"), temp_timeout);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  timeout_ = temp_timeout;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
