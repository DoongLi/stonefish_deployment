/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/captain_helper.h"

namespace cola2
{
namespace rosutils
{
void waitForIdleHelper(bool* is_idle, bool* is_safety, std::mutex* mtx, bool* first_call,
                       const cola2_msgs::CaptainStatus::ConstPtr& captain_status)
{
  std::lock_guard<std::mutex> guard(*mtx);
  if (*first_call)
  {
    (*first_call) = false;
    return;  // Discard first message
  }
  (*is_idle) = (captain_status->state == cola2_msgs::CaptainStatus::IDLE);
  (*is_safety) = (captain_status->state == cola2_msgs::CaptainStatus::SAFETYKEEPPOSITION);
}

bool waitForIdle(ros::NodeHandle& nh)
{
  bool is_idle(false);
  bool is_safety(false);
  std::mutex mtx;
  bool first_call(true);
  ros::CallbackQueue queue;
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<cola2_msgs::CaptainStatus>(
      ros::this_node::getNamespace() + "/captain/captain_status", 10,
      boost::bind(&waitForIdleHelper, &is_idle, &is_safety, &mtx, &first_call, _1), ros::VoidPtr(), &queue);
  ros::Subscriber sub = nh.subscribe(ops);
  ros::AsyncSpinner async_spinner(1, &queue);
  async_spinner.start();
  while (!ros::isShuttingDown())
  {
    {
      std::lock_guard<std::mutex> guard(mtx);
      if (is_safety)
        return false;
      if (is_idle)
        return true;
    }
    ros::Duration(0.1).sleep();
  }
  return false;
}
}  // namespace rosutils
}  // namespace cola2
