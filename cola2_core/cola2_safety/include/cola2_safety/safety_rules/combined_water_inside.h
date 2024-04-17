/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMBINED_WATER_INSIDE_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMBINED_WATER_INSIDE_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class CombinedWaterInside : public SafetyRuleBaseClass
{
 protected:
  bool check_battery_housing_;
  bool check_extra_housing_;
  bool water_battery_external_;
  bool water_battery_internal_;
  bool water_main_external_;
  bool water_main_internal_;
  bool water_extra_;

 public:
  explicit CombinedWaterInside(const std::string&);
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
  void updateParseList();
  void parseDiagnostics();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMBINED_WATER_INSIDE_H_
