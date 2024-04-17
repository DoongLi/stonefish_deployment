/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <actionlib/server/simple_action_server.h>
#include <cola2_control/controllers/anchor.h>
#include <cola2_control/controllers/holonomic_keep_position.h>
#include <cola2_control/controllers/section.h>
#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/ned.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/PilotAction.h>
#include <cola2_msgs/WorldWaypointReq.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

#include <boost/bind.hpp>  // Because of the actionlib callback...
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

class Pilot
{
protected:
  // Node handle
  ros::NodeHandle nh_;

  // ROS variables
  ros::Subscriber sub_nav_;
  ros::Publisher pub_wwr_;
  ros::Publisher pub_bvr_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_goal_;
  ros::ServiceServer srv_reload_params_;
  ros::ServiceClient srv_publish_params_;
  ros::Timer diagnostics_timer_;
  cola2::ros::DiagnosticHelper diagnostic_;

  // Actionlib server
  std::shared_ptr<actionlib::SimpleActionServer<cola2_msgs::PilotAction> > pilot_server_;

  // Current state
  control::State current_state_;
  double last_nav_received_;

  // Controllers
  std::shared_ptr<SectionController> section_controller_;
  std::shared_ptr<HolonomicKeepPositionController> holonomic_keep_position_controller_;
  std::shared_ptr<AnchorController> anchor_controller_;

  // Config
  struct
  {
    SectionControllerConfig section_config;
    HolonomicKeepPositionControllerConfig holonomic_keep_position_config;
    AnchorControllerConfig anchor_config;
  } config_;

  // Methods
  /**
   * \brief Diagnostics timer
   */
  void diagnosticsTimer(const ros::TimerEvent&);

  /**
   * Callback to topic VEHICLE_NAMESPACE/navigator/navigation
   */
  void navCallback(const cola2_msgs::NavSts&);

  /**
   * Callback for actionlib Pilot
   */
  void pilotServerCallback(const cola2_msgs::PilotGoalConstPtr&);

  /**
   * Helper method to publish control commands: WorldWaypointReq and BodyForceReq
   */
  void publishControlCommands(const control::State&, const std::uint64_t, const ros::Time&) const;

  /**
   * Publish feedback for actionlibs
   */
  void publishFeedback(const control::Feedback&) const;

  /**
   * Publish an RViz marker to the direction that the AUV is going
   */
  void publishMarker(const double, const double, const double) const;

  /**
   * Publishes a Section RViz marker
   */
  void publishMarkerSections(const control::PointsList) const;

  /**
   * Load parameters from ROS param server
   */
  void getConfig();

  /**
   * Service to reload parameters from ROS param server
   */
  bool reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

  /**
   * Publishes the waypoint that the AUV is going to as a geometry_msgs::PointStamped
   * REDUNDANT WITH publishMarker??
   */
  void publishGoal(const double, const double, const double) const;

public:
  /**
   * Class constructor
   */
  Pilot();
};

Pilot::Pilot() : nh_("~"), diagnostic_(nh_, "pilot", cola2::ros::getUnresolvedNodeName()), last_nav_received_(0.0)
{
  // Wait for time
  while (ros::Time::now().toSec() == 0.0)
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(1.0, "Waiting for valid time source");
  }

  // Get config
  getConfig();

  // Initialize controllers
  section_controller_ = std::make_shared<SectionController>(config_.section_config);
  holonomic_keep_position_controller_ =
      std::make_shared<HolonomicKeepPositionController>(config_.holonomic_keep_position_config);
  anchor_controller_ = std::make_shared<AnchorController>(config_.anchor_config);

  // Reload parameters service
  srv_reload_params_ = nh_.advertiseService("reload_params", &Pilot::reloadParamsCallback, this);

  // Publishers
  pub_wwr_ =
      nh_.advertise<cola2_msgs::WorldWaypointReq>(cola2::ros::getNamespace() + "/controller/world_waypoint_req", 1);
  pub_bvr_ =
      nh_.advertise<cola2_msgs::BodyVelocityReq>(cola2::ros::getNamespace() + "/controller/body_velocity_req", 1);
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
  pub_goal_ = nh_.advertise<geometry_msgs::PointStamped>("goal", 1, true);

  // Subscriber
  sub_nav_ = nh_.subscribe(cola2::ros::getNamespace() + "/navigator/navigation", 1, &Pilot::navCallback, this);

  // Service client to publish parameters
  std::string publish_params_srv_name = cola2::ros::getNamespace() + "/param_logger/publish_params";
  srv_publish_params_ = nh_.serviceClient<std_srvs::Trigger>(publish_params_srv_name);
  while ((!ros::isShuttingDown()) && (!srv_publish_params_.waitForExistence(ros::Duration(5.0))))
  {
    ROS_INFO_STREAM("Waiting for client to service " << publish_params_srv_name);
  }

  // Actionlib server. Smart pointer is used so that server construction is delayed after configuration is loaded
  pilot_server_ = std::make_shared<actionlib::SimpleActionServer<cola2_msgs::PilotAction> >(
      nh_, "actionlib", boost::bind(&Pilot::pilotServerCallback, this, _1), false);
  pilot_server_->start();

  // Diagnostics timer
  diagnostics_timer_ = nh_.createTimer(ros::Duration(0.5), &Pilot::diagnosticsTimer, this);

  diagnostic_.setEnabled(true);
  ROS_INFO_STREAM("Initialized");
}

void Pilot::diagnosticsTimer(const ros::TimerEvent& event)
{
  diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  diagnostic_.publish(event.current_real);
}

void Pilot::navCallback(const cola2_msgs::NavSts& data)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(data))
  {
    return;
  }

  // Obtain navigation data
  current_state_.pose.position.ned_origin_latitude = data.origin.latitude;
  current_state_.pose.position.ned_origin_longitude = data.origin.longitude;
  current_state_.pose.position.north = data.position.north;
  current_state_.pose.position.east = data.position.east;
  current_state_.pose.position.depth = data.position.depth;
  current_state_.pose.altitude = data.altitude;
  current_state_.pose.orientation.roll = data.orientation.roll;
  current_state_.pose.orientation.pitch = data.orientation.pitch;
  current_state_.pose.orientation.yaw = data.orientation.yaw;
  current_state_.velocity.linear.x = data.body_velocity.x;
  current_state_.velocity.linear.y = data.body_velocity.y;
  current_state_.velocity.linear.z = data.body_velocity.z;
  last_nav_received_ = data.header.stamp.toSec();
}

void Pilot::pilotServerCallback(const cola2_msgs::PilotGoalConstPtr& data)
{
  // Copy most of the input data to the request data type
  control::Request request;
  request.initial_depth = data->initial_depth;
  request.final_depth = data->final_depth;
  request.final_yaw = data->final_yaw;
  request.final_altitude = data->final_altitude;
  if (data->heave_mode == cola2_msgs::PilotGoal::DEPTH)
    request.heave_mode = control::Request::DEPTH;
  else if (data->heave_mode == cola2_msgs::PilotGoal::ALTITUDE)
    request.heave_mode = control::Request::ALTITUDE;
  else if (data->heave_mode == cola2_msgs::PilotGoal::BOTH)
    request.heave_mode = control::Request::BOTH;
  else
  {
    ROS_ERROR_STREAM("Unable to process actionlib request. Unknown heave mode: " << data->heave_mode);
    cola2_msgs::PilotResult result_msg;
    result_msg.state = cola2_msgs::PilotResult::FAILURE;
    pilot_server_->setAborted(result_msg);
    return;
  }
  request.surge_velocity = data->surge_velocity;
  request.tolerance_xy = data->tolerance_xy;
  request.timeout = data->timeout;
  if (data->controller_type == cola2_msgs::PilotGoal::SECTION)
    request.controller_type = control::Request::SECTION;
  else if (data->controller_type == cola2_msgs::PilotGoal::ANCHOR)
    request.controller_type = control::Request::ANCHOR;
  else if (data->controller_type == cola2_msgs::PilotGoal::HOLONOMIC_KEEP_POSITION)
    request.controller_type = control::Request::HOLONOMIC_KEEP_POSITION;
  else
  {
    ROS_ERROR_STREAM("Unable to process actionlib request. Unknown controller type: " << data->controller_type);
    cola2_msgs::PilotResult result_msg;
    result_msg.state = cola2_msgs::PilotResult::FAILURE;
    pilot_server_->setAborted(result_msg);
    return;
  }
  request.requester = data->goal.requester;
  request.priority = data->goal.priority;

  // Main loop
  const double init_time = ros::Time::now().toSec();
  ros::Rate r(10);
  while (!ros::isShuttingDown())
  {
    // Get iteration time stamp
    const ros::Time iteration_stamp = ros::Time::now();

    // Check for preempted. This happens upon user request (by preempting
    // or canceling the goal, or when a new goal is received
    if (pilot_server_->isPreemptRequested())
    {
      ROS_INFO_STREAM("Preempted");
      pilot_server_->setPreempted();
      return;
    }

    // Check timeout
    if (iteration_stamp.toSec() - init_time > request.timeout)
    {
      ROS_WARN_STREAM("Timeout");
      cola2_msgs::PilotResult result_msg;
      result_msg.state = cola2_msgs::PilotResult::TIMEOUT;
      pilot_server_->setAborted(result_msg);
      return;
    }

    // Check navigation
    if (iteration_stamp.toSec() - last_nav_received_ > 2.0)
    {
      ROS_ERROR_STREAM("Pilot actionlib failed due to missing navigation");
      cola2_msgs::PilotResult result_msg;
      result_msg.state = cola2_msgs::PilotResult::FAILURE;
      pilot_server_->setAborted(result_msg);
      return;
    }

    // Now that we know that we have recent navigation, convert from latitude and longitude to north and east
    cola2::utils::NED ned(current_state_.pose.position.ned_origin_latitude,
                          current_state_.pose.position.ned_origin_longitude, 0.0);
    double dummy_depth;
    ned.geodetic2Ned(data->initial_latitude, data->initial_longitude, 0.0, request.initial_north, request.initial_east,
                     dummy_depth);
    ned.geodetic2Ned(data->final_latitude, data->final_longitude, 0.0, request.final_north, request.final_east,
                     dummy_depth);
    request.ned_origin_latitude = current_state_.pose.position.ned_origin_latitude;
    request.ned_origin_longitude = current_state_.pose.position.ned_origin_longitude;

    // Declare the output of the controllers
    control::State controller_output;
    control::Feedback feedback;
    control::PointsList points;

    // Run controller
    try
    {
      if (request.controller_type == control::Request::SECTION)
        section_controller_->compute(current_state_, request, controller_output, feedback, points);
      else if (request.controller_type == control::Request::ANCHOR)
        anchor_controller_->compute(current_state_, request, controller_output, feedback, points);
      else
        holonomic_keep_position_controller_->compute(current_state_, request, controller_output, feedback, points);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("Controller failure: " << ex.what());
      cola2_msgs::PilotResult result_msg;
      result_msg.state = cola2_msgs::PilotResult::FAILURE;
      pilot_server_->setAborted(result_msg);
      return;
    }

    // Publish
    publishControlCommands(controller_output, request.priority, iteration_stamp);
    publishFeedback(feedback);
    publishMarker(request.final_north, request.final_east, request.final_depth);
    publishMarkerSections(points);
    publishGoal(request.final_north, request.final_east, request.final_depth);

    // Check for success
    if (feedback.success)
    {
      ROS_INFO_STREAM("Success");
      cola2_msgs::PilotResult result_msg;
      result_msg.state = cola2_msgs::PilotResult::SUCCESS;
      pilot_server_->setSucceeded(result_msg);
      break;
    }

    // Diagnostic
    diagnostic_.reportValidData(iteration_stamp);

    // Sleep
    r.sleep();
  }
}

void Pilot::publishGoal(const double x, const double y, const double z) const
{
  geometry_msgs::PointStamped goal;
  goal.header.frame_id = "world_ned";
  goal.header.stamp = ros::Time::now();
  goal.point.x = x;
  goal.point.y = y;
  goal.point.z = z;
  pub_goal_.publish(goal);
}

void Pilot::publishControlCommands(const control::State& controller_output, const std::uint64_t priority,
                                   const ros::Time& now) const
{
  // Create and publish world waypoint request
  cola2_msgs::WorldWaypointReq wwr;
  wwr.header.frame_id = "world_ned";
  wwr.header.stamp = now;
  wwr.goal.priority = priority;
  wwr.goal.requester = ros::this_node::getName() + "_pose_req";
  wwr.disable_axis.x = controller_output.pose.disable_axis.x;
  wwr.disable_axis.y = controller_output.pose.disable_axis.y;
  wwr.disable_axis.z = controller_output.pose.disable_axis.z;
  wwr.disable_axis.roll = controller_output.pose.disable_axis.roll;
  wwr.disable_axis.pitch = controller_output.pose.disable_axis.pitch;
  wwr.disable_axis.yaw = controller_output.pose.disable_axis.yaw;
  wwr.position.north = controller_output.pose.position.north;
  wwr.position.east = controller_output.pose.position.east;
  wwr.position.depth = controller_output.pose.position.depth;
  wwr.orientation.roll = controller_output.pose.orientation.roll;
  wwr.orientation.pitch = controller_output.pose.orientation.pitch;
  wwr.orientation.yaw = controller_output.pose.orientation.yaw;
  wwr.altitude_mode = controller_output.pose.altitude_mode;
  wwr.altitude = controller_output.pose.altitude;
  pub_wwr_.publish(wwr);

  // Create and publish body velocity request
  cola2_msgs::BodyVelocityReq bvr;
  bvr.header.frame_id = cola2::ros::getNamespaceNoInitialDash() + "/base_link";
  bvr.header.stamp = now;
  bvr.goal.priority = priority;
  bvr.goal.requester = ros::this_node::getName() + "_velocity_req";
  bvr.disable_axis.x = controller_output.velocity.disable_axis.x;
  bvr.disable_axis.y = controller_output.velocity.disable_axis.y;
  bvr.disable_axis.z = controller_output.velocity.disable_axis.z;
  bvr.disable_axis.roll = controller_output.velocity.disable_axis.roll;
  bvr.disable_axis.pitch = controller_output.velocity.disable_axis.pitch;
  bvr.disable_axis.yaw = controller_output.velocity.disable_axis.yaw;
  bvr.twist.linear.x = controller_output.velocity.linear.x;
  bvr.twist.linear.y = controller_output.velocity.linear.y;
  bvr.twist.linear.z = controller_output.velocity.linear.z;
  bvr.twist.angular.x = controller_output.velocity.angular.x;
  bvr.twist.angular.y = controller_output.velocity.angular.y;
  bvr.twist.angular.z = controller_output.velocity.angular.z;
  pub_bvr_.publish(bvr);
}

void Pilot::publishFeedback(const control::Feedback& feedback) const
{
  cola2_msgs::PilotFeedback msg;
  msg.distance_to_end = feedback.distance_to_end;
  msg.cross_track_error = feedback.cross_track_error;
  pilot_server_->publishFeedback(msg);
}

void Pilot::publishMarker(const double north, const double east, const double depth) const
{
  // Publish marker. Marker is published periodically so that RViz always
  // receives it, even if RViz is started after the ActionGoal arrives
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_ned";
  marker.header.stamp = ros::Time::now();
  marker.ns = ros::this_node::getName();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = north;
  marker.pose.position.y = east;
  marker.pose.position.z = depth;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(2.0);
  marker.frame_locked = false;
  pub_marker_.publish(marker);
}

void Pilot::publishMarkerSections(const control::PointsList points) const
{
  // Create visualization marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_ned";
  marker.header.stamp = ros::Time::now();
  marker.ns = ros::this_node::getName();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  // Add points to it
  for (const auto& i : points.points_list)
  {
    geometry_msgs::Point p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    marker.points.push_back(p);
  }

  marker.scale.x = 0.35;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(2.0);
  marker.frame_locked = false;
  pub_marker_.publish(marker);
}

void Pilot::getConfig()
{
  // Load config from param server
  // clang-format off
  // LOS-CTE controller
  cola2::ros::getParam("~section/tolerance_z", config_.section_config.tolerance_z, 1.0);
  cola2::ros::getParam("~section/delta", config_.section_config.delta, 5.0);
  cola2::ros::getParam("~section/distance_to_max_velocity", config_.section_config.distance_to_max_velocity, 5.0);
  cola2::ros::getParam("~section/max_surge_velocity", config_.section_config.max_surge_velocity, 0.5);
  cola2::ros::getParam("~section/min_surge_velocity", config_.section_config.min_surge_velocity, 0.2);

  // ANCHOR controller
  cola2::ros::getParam("~anchor/kp", config_.anchor_config.kp, 0.1);
  cola2::ros::getParam("~anchor/radius", config_.anchor_config.radius, 1.0);
  cola2::ros::getParam("~anchor/min_surge_velocity", config_.anchor_config.min_surge_velocity, -0.1);
  cola2::ros::getParam("~anchor/max_surge_velocity", config_.anchor_config.max_surge_velocity, 0.3);
  // clang-format on
}

bool Pilot::reloadParamsCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  getConfig();
  section_controller_->setConfig(config_.section_config);
  holonomic_keep_position_controller_->setConfig(config_.holonomic_keep_position_config);
  anchor_controller_->setConfig(config_.anchor_config);

  // Publish params after param reload
  std_srvs::Trigger trigger;
  srv_publish_params_.call(trigger);
  if (trigger.response.success)
  {
    res.message = "Params reloaded";
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    res.message = "Params reloaded, but publish params service did not succeed: " + trigger.response.message;
    ROS_WARN_STREAM(res.message);
  }
  res.success = trigger.response.success;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pilot");
  Pilot pilot;
  ros::spin();
  return 0;
}
