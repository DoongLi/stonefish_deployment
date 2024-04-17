/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CAPTAIN_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CAPTAIN_H_

#include <cola2_safety/safety_rules/common.h>
#include <cola2_msgs/CaptainStateFeedback.h>

namespace SafetyRules
{
class Captain : public SafetyRuleBaseClass
{
 protected:
  ros::Subscriber sub_captain_;
  std::uint32_t current_step_;
  double last_curent_step_;

  // Methods
  void captainStateFeedbackCallback(const cola2_msgs::CaptainStateFeedback&);

 public:
  explicit Captain(const std::string&, ros::NodeHandle*);
  void periodicUpdate(const ros::Time&, std::uint32_t*);
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_CAPTAIN_H_
