/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATER_INSIDE_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATER_INSIDE_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class WaterInside : public SafetyRuleBaseClass
{
 protected:
  bool water_inside_;

 public:
  explicit WaterInside(const std::string&);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATER_INSIDE_H_
