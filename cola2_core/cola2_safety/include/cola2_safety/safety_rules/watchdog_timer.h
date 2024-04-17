/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATCHDOG_TIMER_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATCHDOG_TIMER_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class WatchdogTimer : public SafetyRuleBaseClass
{
 protected:
  double elapsed_time_, timeout_;

 public:
  explicit WatchdogTimer(const std::string&);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_WATCHDOG_TIMER_H_
