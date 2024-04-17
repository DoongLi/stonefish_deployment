/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <ros/ros.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <cola2_msgs/Setpoints.h>
#include <algorithm>
#include <string>
#include <vector>

class SafetySupervisorChecker
{
 protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_thrusters_;
  ros::Subscriber sub_sss_;
  ros::Timer main_timer_;
  cola2::ros::DiagnosticHelper diagnostic_;
  std::string ns_;
  double last_sss_;
  double last_emergency_surface_;
  std::vector<double> emergency_surface_setpoints_;

  // Methods
  void sssCallback(const cola2_msgs::SafetySupervisorStatus&);
  void mainTimerCallback(const ros::TimerEvent&);

 public:
  SafetySupervisorChecker();
};

SafetySupervisorChecker::SafetySupervisorChecker()
  : nh_("~")
  , diagnostic_(nh_, "safety_supervisor_checker", cola2::ros::getUnresolvedNodeName())
  , ns_(cola2::ros::getNamespace())
  , last_emergency_surface_(0.0)
{
  // Wait for time and initialize variables
  while (ros::Time::now().toSec() == 0.0)
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }
  last_sss_ = ros::Time::now().toSec();

  // Get config
  bool ok = true;
  ok &= cola2::ros::getParam(ns_ + "/safety_supervisor/emergency_surface_setpoints", emergency_surface_setpoints_);
  if (!ok)
  {
    ROS_FATAL_STREAM("Wrong or missing basic configuration. Shutting down");
    ros::shutdown();
  }

  // Publishers
  pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>(ns_ + "/controller/thruster_setpoints", 1, true);

  // Subscriber
  sub_sss_ = nh_.subscribe(ns_ + "/safety_supervisor/status", 1, &SafetySupervisorChecker::sssCallback, this);

  // Main timer
  main_timer_ = nh_.createTimer(ros::Duration(0.1), &SafetySupervisorChecker::mainTimerCallback, this);

  diagnostic_.setEnabled(true);
  ROS_INFO("Initialized");
}

void
SafetySupervisorChecker::sssCallback(const cola2_msgs::SafetySupervisorStatus&)
{
  last_sss_ = ros::Time::now().toSec();
}

void
SafetySupervisorChecker::mainTimerCallback(const ros::TimerEvent&)
{
  diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  const ros::Time now = ros::Time::now();
  if (now.toSec() - last_sss_ > 20.0)
  {
    if (last_emergency_surface_ == 0.0)
      last_emergency_surface_ = now.toSec();
    cola2_msgs::Setpoints msg_setpoints;
    msg_setpoints.header.stamp = now;  // Safer than using event
    msg_setpoints.header.frame_id = "safety";
    const double factor = std::max(0.0, std::min(1.0, (msg_setpoints.header.stamp.toSec() - last_emergency_surface_) / 30.0));
    msg_setpoints.setpoints.reserve(emergency_surface_setpoints_.size());
    for (const auto& setpoint : emergency_surface_setpoints_)
      msg_setpoints.setpoints.push_back(factor * setpoint);
    pub_thrusters_.publish(msg_setpoints);
    std::string error_msg("Safety supervisor is not publishing the status message. Sending emergency setpoints");
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg);
    ROS_ERROR_STREAM_THROTTLE(1.0, error_msg);
  }
  else
    last_emergency_surface_ = 0.0;
  diagnostic_.reportValidData(now);
  diagnostic_.publish(now);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_supervisor_checker");
  SafetySupervisorChecker safety_supervisor_checker;
  ros::spin();
  return 0;
}
