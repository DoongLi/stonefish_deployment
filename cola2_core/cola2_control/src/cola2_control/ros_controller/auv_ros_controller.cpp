
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/ros_controller/auv_ros_controller_base.h>
#include <cola2_lib_ros/navigation_helper.h>

IAUVROSController::IAUVROSController(const std::string name, const std::string frame_id)
  : nh_("~")
  , name_(name)
  ,  // TODO: remove this
  frame_id_(frame_id)
  , diagnostic_(nh_, name, "soft")
  , last_nav_time_(0.0)
  , last_altitude_(0.5)
  , last_altitude_age_(0.0)
  , last_depth_(0.0)
{
}

void IAUVROSController::initBase(std::shared_ptr<IAUVController> auv_controller_ptr, double period)
{
  // Init pointer to AUV controller
  auv_controller_ = auv_controller_ptr;

  // Save controller frequency
  frequency_ = 1.0 / period;

  // Publishers
  pub_wrench_ = nh_.advertise<cola2_msgs::BodyForceReq>("merged_body_force_req", 1);
  pub_merged_pose_ = nh_.advertise<cola2_msgs::WorldWaypointReq>("merged_world_waypoint_req", 1);
  pub_merged_twist_ = nh_.advertise<cola2_msgs::BodyVelocityReq>("merged_body_velocity_req", 1);
  pub_thrusters_setpoint_ = nh_.advertise<cola2_msgs::Setpoints>("thruster_setpoints", 1);

  // Subscribers --> WARNING! The buffer should be at least the size of maximum Request send per iteration/kind
  sub_nav_data_ =
      nh_.subscribe(cola2::ros::getNamespace() + "/navigator/navigation", 2, &IAUVROSController::updateNav, this);
  sub_ww_req_ = nh_.subscribe("world_waypoint_req", 10, &IAUVROSController::updateWWR, this);
  sub_bv_req_ = nh_.subscribe("body_velocity_req", 10, &IAUVROSController::updateBVR, this);
  sub_bf_req_ = nh_.subscribe("body_force_req", 10, &IAUVROSController::updateBFR, this);

  _are_thrusters_killed = false;

  // Services
  enable_pose_controller_srv_ =
      nh_.advertiseService("enable_pose_controller", &IAUVROSController::enablePoseController, this);
  disable_pose_controller_srv_ =
      nh_.advertiseService("disable_pose_controller", &IAUVROSController::disablePoseController, this);
  enable_velocity_controller_srv_ =
      nh_.advertiseService("enable_velocity_controller", &IAUVROSController::enableVelocityController, this);
  disable_velocity_controller_srv_ =
      nh_.advertiseService("disable_velocity_controller", &IAUVROSController::disableVelocityController, this);
  enable_thruster_allocator_srv_ =
      nh_.advertiseService("enable_thrusters", &IAUVROSController::enableThrusterAllocator, this);
  disable_thruster_allocator_srv_ =
      nh_.advertiseService("disable_thrusters", &IAUVROSController::disableThrusterAllocator, this);

  // Timers
  timer_ = nh_.createTimer(ros::Duration(period), &IAUVROSController::timerCallback, this);
  check_diagnostics_ = nh_.createTimer(ros::Duration(1.0), &IAUVROSController::checkDiagnostics, this);

  diagnostic_.setEnabled(true);
}

bool IAUVROSController::enablePoseController(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Enable pose controller\n";
  auv_controller_->setPoseController(true);
  res.success = true;
  return true;
}

bool IAUVROSController::disablePoseController(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Disable pose controller\n";
  auv_controller_->setPoseController(false);
  res.success = true;
  return true;
}

bool IAUVROSController::enableVelocityController(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Enable velocity controller\n";
  auv_controller_->setVelocityController(true);
  res.success = true;
  return true;
}

bool IAUVROSController::disableVelocityController(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Disable velocity controller\n";
  auv_controller_->setVelocityController(false);
  res.success = true;
  return true;
}

bool IAUVROSController::enableThrusterAllocator(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Enable thruster allocator";
  auv_controller_->setThrusterAllocator(true);
  _are_thrusters_killed = false;
  res.success = true;
  return true;
}

bool IAUVROSController::disableThrusterAllocator(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << "[Controller] Disable thruster allocator";

  // Send last setpoint to zero
  Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
  for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
  {
    setpoint(i) = 0.0;
  }
  publishThrusterSetpoint(setpoint, ros::Time::now());

  // Disable thrusters
  auv_controller_->setThrusterAllocator(false);
  res.success = true;
  return true;
}

void IAUVROSController::checkDiagnostics(const ros::TimerEvent& event)
{
  diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  if (!auv_controller_->isPoseControllerEnable())
  {
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "pose controller disabled");
  }
  if (!auv_controller_->isVelocityControllerEnable())
  {
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "velocity controller disabled");
  }
  if (!auv_controller_->isThrusterAllocatorEnable())
  {
    diagnostic_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "thruster allocator disabled");
  }
  diagnostic_.addKeyValue("pose_controller_enabled", auv_controller_->isPoseControllerEnable());
  diagnostic_.addKeyValue("velocity_controller_enabled", auv_controller_->isVelocityControllerEnable());
  diagnostic_.addKeyValue("thruster_allocator_enabled", auv_controller_->isThrusterAllocatorEnable());
  diagnostic_.publish(event.current_real);
}

void IAUVROSController::timerCallback(const ros::TimerEvent& event)
{
  // Get current time
  ros::Time now = ros::Time::now();

  // Check last navigation time
  if (std::fabs(now.toSec() - last_nav_time_) > 1.0)
  {
    ROS_INFO_THROTTLE(5.0, "Waiting for navigation");
    diagnostic_.reportData(event.current_real);

    // Send zeros
    Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
    for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
    {
      setpoint(i) = 0.0;
    }
    publishThrusterSetpoint(setpoint, now);
    return;
  }

  // Iterate controller
  auv_controller_->iteration(now.toSec());

  // Compute thruster setpoints
  auv_controller_->computeThrusterAllocator();

  // Check period for diagnostics
  diagnostic_.reportValidData(event.current_real);

  // Publish data
  publishMergedPose(auv_controller_->getMergedPose(), now);
  publishMergedTwist(auv_controller_->getMergedTwist(), now);
  publishMergedWrench(auv_controller_->getMergedWrench(), now);

  // Publish thruster setpoint if enabled
  if (auv_controller_->isThrusterAllocatorEnable())
  {
    Eigen::VectorXd setpoint = auv_controller_->getThrusterSetpoints();
    publishThrusterSetpoint(setpoint, now);
  }
  else
  {
    // Send zeros
    Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
    for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
    {
      setpoint(i) = 0.0;
    }
    publishThrusterSetpoint(setpoint, now);
  }
}

void IAUVROSController::updateNav(const ros::MessageEvent<cola2_msgs::NavSts const>& msg)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(*msg.getMessage()))
  {
    return;
  }

  // Store last valid navigation time
  last_nav_time_ = msg.getMessage()->header.stamp.toSec();

  // Update pose feedback
  std::vector<double> pose_feedback;
  pose_feedback.push_back(msg.getMessage()->position.north);
  pose_feedback.push_back(msg.getMessage()->position.east);
  pose_feedback.push_back(msg.getMessage()->position.depth);
  pose_feedback.push_back(msg.getMessage()->orientation.roll);
  pose_feedback.push_back(msg.getMessage()->orientation.pitch);
  pose_feedback.push_back(msg.getMessage()->orientation.yaw);
  auv_controller_->updatePoseFeedback(pose_feedback);

  // Update twist feedback
  std::vector<double> twist_feedback;
  twist_feedback.push_back(msg.getMessage()->body_velocity.x);
  twist_feedback.push_back(msg.getMessage()->body_velocity.y);
  twist_feedback.push_back(msg.getMessage()->body_velocity.z);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.roll);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.pitch);
  twist_feedback.push_back(msg.getMessage()->orientation_rate.yaw);
  auv_controller_->updateTwistFeedback(twist_feedback);

  // Stores last altitude. If altitude is invalid, during 5 seconds estimate it wrt last altitude and delta depth.
  // If more than 5 seconds put it a 0.5.
  if (msg.getMessage()->altitude > 0.0)
  {
    last_altitude_ = msg.getMessage()->altitude;
    last_altitude_age_ = msg.getMessage()->header.stamp.toSec();
    last_depth_ = msg.getMessage()->position.depth;
  }
  else
  {
    if ((ros::Time::now().toSec() - last_altitude_age_) > 5.0)
    {
      last_altitude_ = 0.5;
    }
    else
    {
      last_altitude_ = last_altitude_ - (msg.getMessage()->position.depth - last_depth_);
      last_depth_ = msg.getMessage()->position.depth;
    }
  }
}

void IAUVROSController::updateWWR(const ros::MessageEvent<cola2_msgs::WorldWaypointReq const>& msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester, msg.getMessage()->header.stamp.toSec(), msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg.getMessage()->position.north);
  values.push_back(msg.getMessage()->position.east);

  // If desired Z is in altitude transform it into depth
  if (msg.getMessage()->altitude_mode)
  {
    double altitude_to_depth = (last_depth_ + last_altitude_) - msg.getMessage()->altitude;
    if (altitude_to_depth < 0.0)
      altitude_to_depth = 0.0;
    values.push_back(altitude_to_depth);
  }
  else
  {
    values.push_back(msg.getMessage()->position.depth);
  }

  values.push_back(msg.getMessage()->orientation.roll);
  values.push_back(msg.getMessage()->orientation.pitch);
  values.push_back(msg.getMessage()->orientation.yaw);
  req.setValues(values);

  // Add request to controller ptr.
  auv_controller_->updatePoseRequest(req);
}

void IAUVROSController::updateBVR(const ros::MessageEvent<cola2_msgs::BodyVelocityReq const>& msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester, msg.getMessage()->header.stamp.toSec(), msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg.getMessage()->twist.linear.x);
  values.push_back(msg.getMessage()->twist.linear.y);
  values.push_back(msg.getMessage()->twist.linear.z);
  values.push_back(msg.getMessage()->twist.angular.x);
  values.push_back(msg.getMessage()->twist.angular.y);
  values.push_back(msg.getMessage()->twist.angular.z);
  req.setValues(values);

  // Add request to controller ptr.
  auv_controller_->updateTwistRequest(req);
}

void IAUVROSController::updateBFR(const ros::MessageEvent<cola2_msgs::BodyForceReq const>& msg)
{
  // Init request
  Request req(msg.getMessage()->goal.requester, msg.getMessage()->header.stamp.toSec(), msg.getMessage()->goal.priority,
              6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis.push_back(msg.getMessage()->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg.getMessage()->wrench.force.x);
  values.push_back(msg.getMessage()->wrench.force.y);
  values.push_back(msg.getMessage()->wrench.force.z);
  values.push_back(msg.getMessage()->wrench.torque.x);
  values.push_back(msg.getMessage()->wrench.torque.y);
  values.push_back(msg.getMessage()->wrench.torque.z);
  req.setValues(values);

  // Add request to controller ptr.
  auv_controller_->updateWrenchRequest(req);
}

void IAUVROSController::publishThrusterSetpoint(const Eigen::VectorXd setpoint, const ros::Time now)
{
  // Create ROS thruster setpoint msg
  cola2_msgs::Setpoints output;

  // Fill header
  output.header.frame_id = frame_id_;
  output.header.stamp = now;

  for (unsigned int i = 0; i < setpoint.size(); i++)
  {
    output.setpoints.push_back(setpoint[i]);
  }

  // Publish message
  pub_thrusters_setpoint_.publish(output);
}

void IAUVROSController::publishMergedPose(const Request pose, const ros::Time now)
{
  // Create ROS output
  cola2_msgs::WorldWaypointReq output;

  // Fill header
  output.header.frame_id = "world_ned";
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = pose.getPriority();
  output.goal.requester = pose.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = pose.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = pose.getValues();
  assert(values.size() == 6);
  output.position.north = values.at(0);
  output.position.east = values.at(1);
  output.position.depth = values.at(2);
  output.orientation.roll = values.at(3);
  output.orientation.pitch = values.at(4);
  output.orientation.yaw = values.at(5);

  // Publish output
  pub_merged_pose_.publish(output);
}

void IAUVROSController::publishMergedTwist(const Request twist, const ros::Time now)
{
  // Create ROS output
  cola2_msgs::BodyVelocityReq output;

  // Fill header
  output.header.frame_id = frame_id_;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = twist.getPriority();
  output.goal.requester = twist.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = twist.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = twist.getValues();
  assert(values.size() == 6);
  output.twist.linear.x = values.at(0);
  output.twist.linear.y = values.at(1);
  output.twist.linear.z = values.at(2);
  output.twist.angular.x = values.at(3);
  output.twist.angular.y = values.at(4);
  output.twist.angular.z = values.at(5);

  // Publish output
  pub_merged_twist_.publish(output);
}

void IAUVROSController::publishMergedWrench(const Request response, const ros::Time now)
{
  // Create ROS output
  cola2_msgs::BodyForceReq output;

  // Fill header
  output.header.frame_id = frame_id_;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = response.getPriority();
  output.goal.requester = response.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = response.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = response.getValues();
  assert(values.size() == 6);
  output.wrench.force.x = values.at(0);
  output.wrench.force.y = values.at(1);
  output.wrench.force.z = values.at(2);
  output.wrench.torque.x = values.at(3);
  output.wrench.torque.y = values.at(4);
  output.wrench.torque.z = values.at(5);

  // Publish output
  pub_wrench_.publish(output);
}
