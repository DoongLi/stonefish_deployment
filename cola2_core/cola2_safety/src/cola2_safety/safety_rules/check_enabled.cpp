/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/check_enabled.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
  CheckEnabled::CheckEnabled(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , level_if_not_enabled_(SafetyLevel::NONE)
{
  loadConfigFromParamServer();
}

void
CheckEnabled::periodicUpdate(const ros::Time& stamp, std::uint32_t*)
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
  else
  {
    std::vector<std::string> missing_diagnostics;
    for (const auto& elem : diagnostic_status_names_to_check_)
    {
      if (!(hasBool(elem) && getBool(elem)))
      {
        missing_diagnostics.push_back(elem);
      }
    }
    if (!missing_diagnostics.empty())
    {
      message_ = createMessage("missing enabled diagnostics: ") + missing_diagnostics[0];
      for (std::size_t i = 1; i < missing_diagnostics.size(); ++i)
        message_ += std::string(", ") + missing_diagnostics[i];
      if (stamp.toSec() - last_valid_config_ < INIT_TIME)
        level_ = SafetyLevel::INFORMATIVE;
      else
        level_ = level_if_not_enabled_;
    }
  }
}

bool
CheckEnabled::loadConfigFromParamServer()
{
  // Load config from param server
  std::vector<std::string> temp_diagnostics_status_names_to_check;
  int temp_level_if_not_enabled;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/diagnostic_status_names_to_check"), temp_diagnostics_status_names_to_check);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/level_if_not_enabled"), temp_level_if_not_enabled);
  if (ok && ((temp_level_if_not_enabled < static_cast<int>(SafetyLevel::NONE)) ||
             (temp_level_if_not_enabled > static_cast<int>(SafetyLevel::DROP_WEIGHT))))
  {
    ok = false;
  }

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  diagnostic_status_names_to_check_ = temp_diagnostics_status_names_to_check;
  level_if_not_enabled_ = static_cast<SafetyLevel::Type>(temp_level_if_not_enabled);
  last_valid_config_ = ros::Time::now().toSec();
  updateParseList();
  return ok;
}

void
CheckEnabled::updateParseList()
{
  ParseList parse_list;
  for (const auto& elem : diagnostic_status_names_to_check_)
  {
    parse_list.push_back({elem, "enabled", elem, DataType::Bool});
  }
  setParseList(parse_list);
}
}  // namespace SafetyRules
