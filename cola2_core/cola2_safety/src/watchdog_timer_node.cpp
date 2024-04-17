/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <string>

class WatchdogTimer
{
 protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_elapsed_;
  ros::ServiceServer srv_reset_timeout_;
  ros::Timer main_timer_;
  cola2::ros::DiagnosticHelper diagnostic_;
  double init_time_;

  // Methods
  bool resetTimeoutCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  void mainTimerCallback(const ros::TimerEvent&);

 public:
  WatchdogTimer();
};

WatchdogTimer::WatchdogTimer()
  : nh_("~")
  , diagnostic_(nh_, "watchdog_timer", cola2::ros::getUnresolvedNodeName())
{
  // Wait for time and initialize variables
  while (ros::Time::now().toSec() == 0.0)
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }
  init_time_ = ros::Time::now().toSec();

  pub_elapsed_ = nh_.advertise<std_msgs::Int32>("elapsed_time", 1, true);
  srv_reset_timeout_ = nh_.advertiseService("reset_timeout", &WatchdogTimer::resetTimeoutCallback, this);
  main_timer_ = nh_.createTimer(ros::Duration(1.0), &WatchdogTimer::mainTimerCallback, this);
  diagnostic_.setEnabled(true);
  ROS_INFO("Initialized");
}

bool
WatchdogTimer::resetTimeoutCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Reset timeout");
  init_time_ = ros::Time::now().toSec();
  res.success = true;
  res.message = "Successful timeout reset";
  return true;
}

void
WatchdogTimer::mainTimerCallback(const ros::TimerEvent& event)
{
  // Publish diagnostic
  diagnostic_.addKeyValue("elapsed_time", event.current_real.toSec() - init_time_);
  diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  diagnostic_.reportValidData(event.current_real);
  diagnostic_.publish(event.current_real);

  // Publish elapsed time message
  std_msgs::Int32 msg;
  msg.data = static_cast<int>(event.current_real.toSec() - init_time_);
  pub_elapsed_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "watchdog_timer");
  WatchdogTimer watchdog_timer;
  ros::spin();
  return 0;
}
