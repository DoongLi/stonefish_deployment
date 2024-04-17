/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TEMPERATURE_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TEMPERATURE_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class Temperature : public SafetyRuleBaseClass
{
 protected:
  std::string diagnostic_status_name_;
  double temperature_, max_temperature_;

 public:
  explicit Temperature(const std::string&);
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
  void updateParseList();
  void parseDiagnostics();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TEMPERATURE_H_
