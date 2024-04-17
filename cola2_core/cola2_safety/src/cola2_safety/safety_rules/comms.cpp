/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/param_loader.h>
#include <cola2_safety/safety_rules/comms.h>

namespace SafetyRules
{
Comms::Comms(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , last_modem_data_(ros::Time::now().toSec())
  , modem_data_timeout_(0.0)
  , modem_recovery_action_(0)
  , diagnostic_status_level_(0)
{
  loadConfigFromParamServer();

  const ParseList parse_list(
      { { std::string("/safety/") + rule_name_, "last_modem_data", "last_modem_data", DataType::Double },
        { std::string("/safety/") + rule_name_, "modem_recovery_action", "modem_recovery_action", DataType::Int },
        { "/safety/modem", DIAGNOSTIC_STATUS_LEVEL, "diagnostic_status_level", DataType::Int } });
  setParseList(parse_list);
}

void Comms::parseDiagnostics()
{
  if (hasInt("modem_recovery_action") && hasInt("diagnostic_status_level"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    modem_recovery_action_ = getInt("modem_recovery_action");
    diagnostic_status_level_ = getInt("diagnostic_status_level");
    if (hasDouble("last_modem_data"))
    {
      last_modem_data_ = getDouble("last_modem_data");
      has_last_modem_data_ = true;  // always true after first apperance
    }
  }
}

void Comms::periodicUpdate(const ros::Time& stamp, std::uint32_t*)
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
    level_ =
        (stamp.toSec() - last_valid_config_ < INIT_TIME ? SafetyLevel::INFORMATIVE : SafetyLevel::ABORT_AND_SURFACE);
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
  else if ((modem_recovery_action_ < static_cast<int>(SafetyLevel::NONE)) ||
           (modem_recovery_action_ > static_cast<int>(SafetyLevel::DROP_WEIGHT)))
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("invalid modem recovery action");
  }
  else if ((has_last_modem_data_) && (last_modem_data_ > modem_data_timeout_) &&
           (modem_recovery_action_ < static_cast<int>(SafetyLevel::ABORT_AND_SURFACE)))
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("modem data age above threshold");
  }
  else if ((diagnostic_status_level_ == diagnostic_msgs::DiagnosticStatus::ERROR) &&
           (modem_recovery_action_ < static_cast<int>(SafetyLevel::ABORT_AND_SURFACE)))
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("modem data error");
  }
  else
  {
    level_ = static_cast<SafetyLevel::Type>(modem_recovery_action_);
    message_ = createMessage("modem recovery action");
  }
}

bool Comms::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_modem_data_timeout;
  bool ok = true;
  ok &=
      cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/modem_data_timeout"), temp_modem_data_timeout);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  modem_data_timeout_ = temp_modem_data_timeout;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
