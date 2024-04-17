/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_MANUAL_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_MANUAL_H_

#include <cola2_safety/safety_rules/common.h>
#include <cola2_msgs/Recovery.h>

namespace SafetyRules
{
class Manual : public SafetyRuleBaseClass
{
 protected:
  ros::ServiceServer srv_;

  // Methods
  bool serviceCallback(cola2_msgs::Recovery::Request&, cola2_msgs::Recovery::Response&);

 public:
  explicit Manual(const std::string&, ros::NodeHandle*);
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_MANUAL_H_
