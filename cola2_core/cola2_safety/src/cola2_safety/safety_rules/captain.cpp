/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/captain.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
  Captain::Captain(const std::string& rule_name, ros::NodeHandle* nh_ptr)
  : SafetyRuleBaseClass(rule_name)
  , current_step_(0)
  , last_curent_step_(ros::Time::now().toSec())
{
  // Subscriber
  sub_captain_ = nh_ptr->subscribe(cola2::ros::getNamespace() + "/captain/state_feedback", 1,
                                   &Captain::captainStateFeedbackCallback, this);
}

void
Captain::captainStateFeedbackCallback(const cola2_msgs::CaptainStateFeedback& msg)
{
  message_.clear();
  current_step_ = 0;
  if (msg.state == cola2_msgs::CaptainStateFeedback::ACTIVE)
  {
    for (const auto& elem : msg.keyvalues)
    {
      if (elem.key.compare("current_step") == 0)
      {
        try
        {
          int current_step = std::stoi(elem.value);
          if ((current_step < 0) || (current_step > 255))
            throw std::runtime_error("Current step out of range");
          current_step_ = static_cast<std::uint32_t>(current_step);
          last_curent_step_ = msg.header.stamp.toSec();
        }
        catch (const std::exception& ex)
        {
          message_ += createMessage("exception while parsing current step: ") + ex.what();
          ROS_ERROR_STREAM(message_);
        }
      }
    }
  }
}

void
Captain::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
{
  // Set level
  level_ = SafetyLevel::NONE;
  if (!message_.empty())
    level_ = SafetyLevel::INFORMATIVE;

  // Clear status code bits for current step
  for (std::size_t i = 0; i < 8; ++i)
    StatusCodeBits::setBit(status_code_ptr, i, false);

  // Check data
  if (stamp.toSec() - last_curent_step_ < INIT_TIME)
    *status_code_ptr += current_step_;
}
}  // namespace SafetyRules
