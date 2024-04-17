/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/water_inside.h>

namespace SafetyRules
{
  WaterInside::WaterInside(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , water_inside_(false)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "water", "water_inside", DataType::Bool}
  });
  setParseList(parse_list);
}

void
WaterInside::parseDiagnostics()
{
  if (hasBool("water_inside"))
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    water_inside_ = getBool("water_inside");
  }
}

void
WaterInside::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
{
  // Clear level and message
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
  else if (water_inside_)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("water inside");
  }

  if ((level_ != SafetyLevel::NONE) && (level_ != SafetyLevel::INFORMATIVE))
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::WATER_INSIDE, true);
}
}  // namespace SafetyRules
