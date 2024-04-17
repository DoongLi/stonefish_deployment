/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/battery_control_board.h>

namespace SafetyRules
{
BatteryControlBoard::BatteryControlBoard(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
{
  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "status_indicator", "status_indicator", DataType::Int}
  });
  setParseList(parse_list);
}

void
BatteryControlBoard::parseDiagnostics()
{
  if (hasInt("status_indicator"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
  }
}

void
BatteryControlBoard::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
{
  // Clear level, message and status code bits
  level_ = SafetyLevel::NONE;
  message_.clear();

  // Check data
  if (last_valid_diagnostics_data_ == 0.0)
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
  else if (hasInt("status_indicator"))
  {
    const int status_indicator = getInt("status_indicator");

    // To have the definitions of these bits, a dependency to the battery control board would be required...
    if (status_indicator == 1)
    {
      message_ = createMessage("switching off due to magnetic switch");
      level_ = SafetyLevel::INFORMATIVE;
      StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::BCB_STATUS_INDICATOR_1, true);
    }
    else if (status_indicator == 2)
    {
      message_ = createMessage("switching off due to BMS error");
      level_ = SafetyLevel::INFORMATIVE;
      StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::BCB_STATUS_INDICATOR_2, true);
    }
    else if (status_indicator == 3)
    {
      message_ = createMessage("switching off due to low battery");
      level_ = SafetyLevel::INFORMATIVE;
      StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::BCB_STATUS_INDICATOR_1, true);
      StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::BCB_STATUS_INDICATOR_2, true);
    }
  }
}
}  // namespace SafetyRules
