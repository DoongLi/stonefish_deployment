/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TELEOPERATION_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TELEOPERATION_H_

#include <cola2_safety/safety_rules/common.h>
#include <cola2_msgs/CaptainStatus.h>

namespace SafetyRules
{
class Teleoperation : public SafetyRuleBaseClass
{
 protected:
  ros::Subscriber sub_captain_;
  double last_ack_;
  double teleoperation_link_timeout_;
  double last_autonomous_;
  double last_captain_status_;

  // Methods
  void captainStatusCallback(const cola2_msgs::CaptainStatus&);

 public:
  explicit Teleoperation(const std::string&, ros::NodeHandle*);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_TELEOPERATION_H_
