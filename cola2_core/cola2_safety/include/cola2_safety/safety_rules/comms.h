/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMS_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMS_H_

#include <cola2_safety/safety_rules/common.h>

namespace SafetyRules
{
class Comms : public SafetyRuleBaseClass
{
protected:
  bool has_last_modem_data_ = false;
  double last_modem_data_, modem_data_timeout_;
  int modem_recovery_action_, diagnostic_status_level_;

public:
  explicit Comms(const std::string&);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMS_H_
