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
#include <cola2_msgs/NavSts.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <exception>
#include <string>

class VirtualCage
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Subscriber sub_nav_;
  ros::ServiceServer srv_reload_params_;
  ros::Timer main_timer_;
  cola2::ros::DiagnosticHelper diagnostic_;
  const unsigned int marker_divisions_;
  double cage_center_north_, cage_center_east_, cage_radius_;
  visualization_msgs::Marker cage_marker_;
  double nav_north_, nav_east_, last_nav_;

  // Methods
  void navCallback(const cola2_msgs::NavSts&);
  bool reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  void mainTimerCallback(const ros::TimerEvent&);
  bool getConfig();

public:
  VirtualCage();
};

VirtualCage::VirtualCage()
  : nh_("~")
  , diagnostic_(nh_, "virtual_cage", cola2::ros::getUnresolvedNodeName())
  , marker_divisions_(180)
  , cage_center_north_(0.0)
  , cage_center_east_(0.0)
  , cage_radius_(0.0)
  , nav_north_(0.0)
  , nav_east_(0.0)
{
  // Wait for time and initialize variables
  while (ros::Time::now().toSec() == 0.0)
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }
  last_nav_ = ros::Time::now().toSec();

  getConfig();
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("cage_marker", 1, true);
  sub_nav_ = nh_.subscribe(cola2::ros::getNamespace() + "/navigator/navigation", 1, &VirtualCage::navCallback, this);
  srv_reload_params_ = nh_.advertiseService("reload_params", &VirtualCage::reloadParamsCallback, this);
  main_timer_ = nh_.createTimer(ros::Duration(1.0), &VirtualCage::mainTimerCallback, this);
  diagnostic_.setEnabled(true);
  ROS_INFO("Initialized");
}

void VirtualCage::navCallback(const cola2_msgs::NavSts& nav)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(nav))
  {
    return;
  }

  nav_north_ = nav.position.north;
  nav_east_ = nav.position.east;
  last_nav_ = nav.header.stamp.toSec();
}

bool VirtualCage::reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
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

void VirtualCage::mainTimerCallback(const ros::TimerEvent& event)
{
  // Update marker
  cage_marker_.header.stamp = event.current_real;
  cage_marker_.color.r = 0.0;
  cage_marker_.color.g = 1.0;

  // Check when the navigation data was received
  if (event.current_real.toSec() - last_nav_ > 30.0)
  {
    ROS_ERROR_STREAM("Navigation data too old");
    cage_marker_.color.r = 1.0;
    cage_marker_.color.g = 0.0;
    diagnostic_.removeKeyValue("distance_to_border");
    diagnostic_.removeKeyValue("inside_virtual_cage");
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN);
  }
  else
  {
    // Check the distance
    double dist = std::sqrt(std::pow(nav_north_ - cage_center_north_, 2) + std::pow(nav_east_ - cage_center_east_, 2));
    diagnostic_.addKeyValue("distance_to_border", std::fabs(dist - cage_radius_));
    if (dist > cage_radius_)
    {
      ROS_WARN_STREAM("Vehicle out of virtual cage");
      diagnostic_.addKeyValue("inside_virtual_cage", false);
      diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN);
      cage_marker_.color.r = 1.0;
      cage_marker_.color.g = 0.0;
    }
    else
    {
      diagnostic_.addKeyValue("inside_virtual_cage", true);
      diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
    }
    diagnostic_.reportValidData(event.current_real);
  }

  // Publish diagnostic
  diagnostic_.publish(event.current_real);

  // Publish marker
  pub_marker_.publish(cage_marker_);
}

bool VirtualCage::getConfig()
{
  // Load configuration
  double temp_cage_center_north, temp_cage_center_east, temp_cage_radius;
  bool ok = true;
  ok &= cola2::ros::getParam("~cage_center_north", temp_cage_center_north);
  ok &= cola2::ros::getParam("~cage_center_east", temp_cage_center_east);
  ok &= cola2::ros::getParam("~cage_radius", temp_cage_radius);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR("Invalid parameters! No changes applied");
    return false;
  }

  // Apply changes
  cage_center_north_ = temp_cage_center_north;
  cage_center_east_ = temp_cage_center_east;
  cage_radius_ = temp_cage_radius;

  // Update marker
  cage_marker_.header.frame_id = "world_ned";
  cage_marker_.ns = "virtual_cage";
  cage_marker_.type = visualization_msgs::Marker::LINE_LIST;
  cage_marker_.action = visualization_msgs::Marker::ADD;
  cage_marker_.pose.orientation.w = 1.0;  // RViz needs it
  cage_marker_.scale.x = 0.5;
  cage_marker_.color.b = 0.0;
  cage_marker_.color.a = 1.0;
  cage_marker_.lifetime = ros::Duration(2.0);
  cage_marker_.frame_locked = false;
  cage_marker_.points.clear();
  for (unsigned int i = 0; i < marker_divisions_; ++i)
  {
    geometry_msgs::Point p;
    p.z = 0.0;
    double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(marker_divisions_);
    p.x = cage_center_north_ + cage_radius_ * std::cos(angle);
    p.y = cage_center_east_ + cage_radius_ * std::sin(angle);
    cage_marker_.points.push_back(p);
    angle = 2.0 * M_PI * static_cast<double>(i + 1) / static_cast<double>(marker_divisions_);
    p.x = cage_center_north_ + cage_radius_ * std::cos(angle);
    p.y = cage_center_east_ + cage_radius_ * std::sin(angle);
    cage_marker_.points.push_back(p);
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "virtual_cage");
  VirtualCage virtual_cage;
  ros::spin();
  return 0;
}
