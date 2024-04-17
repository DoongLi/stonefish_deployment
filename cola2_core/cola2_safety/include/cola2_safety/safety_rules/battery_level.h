/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_BATTERY_LEVEL_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_BATTERY_LEVEL_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class BatteryLevel : public SafetyRuleBaseClass
{
 protected:
  double charge_, voltage_, min_charge_, min_voltage_;

 public:
  static constexpr double LOW_CHARGE_FACTOR = 1.5;
  static constexpr double LOW_VOLTAGE_OFFSET = 1.0;

  explicit BatteryLevel(const std::string&);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_BATTERY_LEVEL_H_
