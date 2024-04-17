/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/serviceclient_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <cola2_msgs/GoalDescriptor.h>
#include <cola2_msgs/NavSts.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <string>

class SafeDepthAltitude
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_bvr_;
  ros::Subscriber sub_nav_;
  ros::ServiceServer srv_enable_no_altitude_goes_up_, srv_disable_no_altitude_goes_up_, srv_reload_params_;
  ros::Timer diagnostics_timer_;
  const std::string ns_;
  cola2::ros::DiagnosticHelper diagnostic_;
  double max_depth_, min_altitude_, min_altitude_starts_at_depth_;
  bool no_altitude_goes_up_;
  double last_nav_received_;
  double initial_time_;

  // Methods
  void navCallback(const cola2_msgs::NavSts&);
  void diagnosticsTimer(const ros::TimerEvent&);
  bool enableNoAltitudeGoesUpCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool disableNoAltitudeGoesUpCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool getConfig();

public:
  SafeDepthAltitude();
};

SafeDepthAltitude::SafeDepthAltitude()
  : nh_("~")
  , ns_(cola2::ros::getNamespace())
  , diagnostic_(nh_, "safe_depth_altitude", cola2::ros::getUnresolvedNodeName())
  , max_depth_(0.0)
  , min_altitude_(10000.0)
  , min_altitude_starts_at_depth_(0.5)
  , no_altitude_goes_up_(true)
  , last_nav_received_(0.0)
{
  // Wait for time and initialize variables
  while ((ros::Time::now().toSec() == 0.0) && (!ros::isShuttingDown()))
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }
  initial_time_ = ros::Time::now().toSec();

  getConfig();
  pub_bvr_ = nh_.advertise<cola2_msgs::BodyVelocityReq>(ns_ + "/controller/body_velocity_req", 1, true);
  sub_nav_ = nh_.subscribe(ns_ + "/navigator/navigation", 10, &SafeDepthAltitude::navCallback, this);
  srv_enable_no_altitude_goes_up_ =
      nh_.advertiseService("enable_no_altitude_goes_up", &SafeDepthAltitude::enableNoAltitudeGoesUpCallback, this);
  srv_disable_no_altitude_goes_up_ =
      nh_.advertiseService("disable_no_altitude_goes_up", &SafeDepthAltitude::disableNoAltitudeGoesUpCallback, this);
  srv_reload_params_ = nh_.advertiseService("reload_params", &SafeDepthAltitude::reloadParamsCallback, this);
  diagnostics_timer_ = nh_.createTimer(ros::Duration(0.5), &SafeDepthAltitude::diagnosticsTimer, this);
  diagnostic_.setEnabled(true);
  ROS_INFO("Initialized");
}

void SafeDepthAltitude::navCallback(const cola2_msgs::NavSts& nav)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(nav))
  {
    return;
  }

  diagnostic_.addKeyValue("altitude", nav.altitude);
  diagnostic_.addKeyValue("depth", nav.position.depth);

  // Check altitude and depth
  bool invalid = false;
  if ((nav.altitude <= 0.0) && no_altitude_goes_up_)
  {
    // TODO: This if () should not be here. Sometimes (mostly in simulation) during initialization the navigator
    // publishes the navigation message even before having a initialized position. When this happens, both
    // position and altitude are EXACTLY zero in the message. Under this circumstances we should not check
    // the altitude. This if () allows for some initialization time
    if (ros::Time::now().toSec() - initial_time_ > 20.0)
    {
      ROS_WARN_STREAM_THROTTLE(1, "No altitude. Going up!");
      invalid = true;
    }
  }
  if ((nav.altitude > 0.0) && (nav.altitude < min_altitude_) && (nav.position.depth > min_altitude_starts_at_depth_))
  {
    ROS_WARN_STREAM_THROTTLE(1,
                             "Invalid altitude (" << nav.altitude << " less than " << min_altitude_ << "). Going up!");
    invalid = true;
  }
  if (nav.position.depth > max_depth_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Invalid depth (" << nav.position.depth << " greater than " << max_depth_
                                                  << "). Going up!");
    invalid = true;
  }

  // If invalid altitude or depth, go up
  if (invalid)
  {
    // Publish body velocity request with high priority. If yaw is disabled the vehicle can rotate while it goes up
    // but if it is enabled, then it can not be teleoperated
    cola2_msgs::BodyVelocityReq bvr;
    bvr.twist.linear.x = 0.0;
    bvr.twist.linear.y = 0.0;
    bvr.twist.linear.z = 0.0;
    if (nav.position.depth > 1.0)  // Only push if the vehicle is submerged to avoid generating bubbles at surface
    {
      bvr.twist.linear.z = -0.5;
    }
    bvr.twist.angular.x = 0.0;
    bvr.twist.angular.y = 0.0;
    bvr.twist.angular.z = 0.0;
    bvr.disable_axis.x = true;
    bvr.disable_axis.y = true;
    bvr.disable_axis.z = false;
    bvr.disable_axis.roll = true;
    bvr.disable_axis.pitch = true;
    bvr.disable_axis.yaw = true;
    bvr.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_SAFETY_HIGH;
    bvr.goal.requester = cola2::ros::getUnresolvedNodeName();
    bvr.header.stamp = ros::Time::now();  // Safer
    pub_bvr_.publish(bvr);
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Invalid depth or altitude");
  }
  else
  {
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  }
  diagnostic_.reportValidData(nav.header.stamp);
  diagnostic_.publish(nav.header.stamp);

  last_nav_received_ = nav.header.stamp.toSec();
}

void SafeDepthAltitude::diagnosticsTimer(const ros::TimerEvent& event)
{
  if (event.current_real.toSec() - last_nav_received_ > 3.0)
  {
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Navigation too old");
    diagnostic_.removeKeyValue("altitude");
    diagnostic_.removeKeyValue("depth");
    diagnostic_.reportData(event.current_real);
    diagnostic_.publish(event.current_real);
  }
}

bool SafeDepthAltitude::enableNoAltitudeGoesUpCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("No altitude goes up reactive behavior enabled");
  no_altitude_goes_up_ = true;
  res.message = "Success";
  res.success = true;
  return true;
}

bool SafeDepthAltitude::disableNoAltitudeGoesUpCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("No altitude goes up reactive behavior disabled");
  no_altitude_goes_up_ = false;
  res.message = "Success";
  res.success = true;
  return true;
}

bool SafeDepthAltitude::reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Reload params service called");

  // Check if something failed
  if (getConfig())
  {
    res.message = "Success";
    res.success = true;

    // Call param logger service
    const std::string srv_name = cola2::ros::getNamespace() + "/param_logger/publish_params";
    try
    {
      std_srvs::Trigger::Request req;
      std_srvs::Trigger::Response res;
      bool success = cola2::ros::callServiceWithTimeout<std_srvs::Trigger>(nh_, req, res, srv_name, 0.5);
      if (!success)
      {
        ROS_ERROR_STREAM("Service " << srv_name << " call failed");
      }
      else
      {
        if (!res.success)
        {
          ROS_WARN_STREAM("Service " << srv_name << " responded False with msg: " << res.message);
        }
      }
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("Exception while calling service " << srv_name << ": " << ex.what());
    }
  }
  else
  {
    res.message = "Invalid parameters";
    res.success = false;
    ROS_ERROR_STREAM(res.message);
  }
  return true;
}

bool SafeDepthAltitude::getConfig()
{
  // Load configuration
  double temp_max_depth, temp_min_altitude, temp_min_altitude_starts_at_depth;
  bool ok = true;
  ok &= cola2::ros::getParam("~max_depth", temp_max_depth);
  ok &= cola2::ros::getParam("~min_altitude", temp_min_altitude);
  ok &= cola2::ros::getParam("~min_altitude_starts_at_depth", temp_min_altitude_starts_at_depth);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR("Invalid parameters! No changes applied");
    return false;
  }

  // Apply changes
  max_depth_ = temp_max_depth;
  min_altitude_ = temp_min_altitude;
  min_altitude_starts_at_depth_ = temp_min_altitude_starts_at_depth;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safe_depth_altitude");
  SafeDepthAltitude safe_depth_altitude;
  ros::spin();
  return 0;
}
