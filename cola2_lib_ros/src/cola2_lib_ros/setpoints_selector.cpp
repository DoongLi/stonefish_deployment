/*
 * Copyright (c) 2024 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/setpoints_selector.h>
#include <cola2_lib_ros/this_node.h>

namespace cola2
{
namespace ros
{
SetpointsSelector::SetpointsSelector(::ros::NodeHandle& nh)
  : priority_list_({ { "radio_open_loop", 0.0 }, { "open_loop", 0.0 }, { "safety", 0.0 } })
{
  pub_ts_ = nh.advertise<cola2_msgs::Setpoints>(cola2::ros::getNamespace() + "/setpoints_selector/thruster_setpoints",
                                                10, true);
}

bool SetpointsSelector::acceptSetpoints(const cola2_msgs::Setpoints& msg)
{
  // The time from the header is not reliable since it may come from different computers
  const double now = ::ros::Time::now().toSec();

  // From higher to lower priority
  for (auto& elem : priority_list_)
  {
    if (msg.header.frame_id.find(std::get<0>(elem)) != msg.header.frame_id.npos)
    {
      // The message frame_id contains the name from the elem of the priority list. Store time and accept the message
      std::get<1>(elem) = now;
      break;
    }
    else if (now - std::get<1>(elem) < EXPIRY_TIME)
    {
      // The frame_id does not match with the name in the elem from the priority list, and there is a recent message of
      // higher priority. Reject the message
      return false;
    }
    // The message frame_id did not match but there are no recent messages of higher priority. Keep trying with next
    // elem or accept it if no more elements in the priority list
  }
  pub_ts_.publish(msg);
  return true;
}
}  // namespace ros
}  // namespace cola2
