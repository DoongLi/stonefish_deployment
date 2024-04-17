/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_VIRTUAL_CAGE_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_VIRTUAL_CAGE_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class VirtualCage : public SafetyRuleBaseClass
{
 protected:
  bool enables_abort_and_surface_;

 public:
  explicit VirtualCage(const std::string&);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_VIRTUAL_CAGE_H_
