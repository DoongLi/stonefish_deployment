/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CHECK_ENABLED_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CHECK_ENABLED_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class CheckEnabled : public SafetyRuleBaseClass
{
 protected:
  std::vector<std::string> diagnostic_status_names_to_check_;
  SafetyLevel::Type level_if_not_enabled_;

 public:
  explicit CheckEnabled(const std::string&);
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
  void updateParseList();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CHECK_ENABLED_H_
