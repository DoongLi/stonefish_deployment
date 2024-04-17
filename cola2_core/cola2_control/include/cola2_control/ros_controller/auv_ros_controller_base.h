
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_

#include <cola2_control/low_level_controllers/auv_controller_base.h>
#include <cola2_control/low_level_controllers/request.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/BodyForceReq.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/Setpoints.h>
#include <cola2_msgs/WorldWaypointReq.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class IAUVROSController
{
private:
  void publishThrusterSetpoint(const Eigen::VectorXd setpoint, const ros::Time now);

  void publishFinSetpoint(const Eigen::VectorXd setpoint, const ros::Time now);

  void publishMergedPose(const Request pose, const ros::Time now);

  void publishMergedTwist(const Request twist, const ros::Time now);

  void publishMergedWrench(const Request response, const ros::Time now);

  // Node handle
  ros::NodeHandle nh_;

  // Name
  std::string name_;

  // Frame id
  std::string frame_id_;

  // Controller frequency
  double frequency_;

  // Diagnostics
  cola2::ros::DiagnosticHelper diagnostic_;

  // Publisher
  ros::Publisher pub_wrench_;
  ros::Publisher pub_merged_pose_;
  ros::Publisher pub_merged_twist_;
  ros::Publisher pub_thrusters_setpoint_;
  ros::Publisher pub_fins_setpoint_;

  // Subscriber
  ros::Subscriber sub_nav_data_;
  ros::Subscriber sub_ww_req_;
  ros::Subscriber sub_bv_req_;
  ros::Subscriber sub_bf_req_;

  bool _are_thrusters_killed;

  // Timers
  ros::Timer timer_;
  ros::Timer check_diagnostics_;

  // Services
  ros::ServiceServer enable_pose_controller_srv_;
  ros::ServiceServer disable_pose_controller_srv_;
  ros::ServiceServer enable_velocity_controller_srv_;
  ros::ServiceServer disable_velocity_controller_srv_;
  ros::ServiceServer enable_thruster_allocator_srv_;
  ros::ServiceServer disable_thruster_allocator_srv_;
  ros::ServiceServer enable_fin_allocator_srv_;
  ros::ServiceServer disable_fin_allocator_srv_;

  // AUV controller ptr.
  std::shared_ptr<IAUVController> auv_controller_;

  // Last navigation time in seconds
  double last_nav_time_;

  // Estimated total altitude
  double last_altitude_;
  double last_altitude_age_;
  double last_depth_;

public:
  IAUVROSController(const std::string name, const std::string frame_id);

  void initBase(std::shared_ptr<IAUVController> auv_controller_ptr, double period);

  bool enablePoseController(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool disablePoseController(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool enableVelocityController(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool disableVelocityController(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool enableThrusterAllocator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool disableThrusterAllocator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool enableFinAllocator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool disableFinAllocator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void checkDiagnostics(const ros::TimerEvent &event);

  void timerCallback(const ros::TimerEvent &event);

  void updateNav(const ros::MessageEvent<cola2_msgs::NavSts const> &msg);

  void updateWWR(const ros::MessageEvent<cola2_msgs::WorldWaypointReq const> &msg);

  void updateBVR(const ros::MessageEvent<cola2_msgs::BodyVelocityReq const> &msg);

  void updateBFR(const ros::MessageEvent<cola2_msgs::BodyForceReq const> &msg);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
