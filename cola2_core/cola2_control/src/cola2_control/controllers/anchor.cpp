/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllers/anchor.h>

// Constructor
AnchorController::AnchorController(AnchorControllerConfig config) : config_(config)
{
}

void AnchorController::setConfig(const AnchorControllerConfig& config)
{
  config_ = config;
}

// Compute Method
void AnchorController::compute(const control::State& current_state, const control::Request& request,
                               control::State& controller_output, control::Feedback& feedback,
                               control::PointsList& marker)
{
  // Set all axis as disabled by default
  controller_output.pose.disable_axis.x = true;
  controller_output.pose.disable_axis.y = true;
  controller_output.pose.disable_axis.z = true;
  controller_output.pose.disable_axis.roll = true;
  controller_output.pose.disable_axis.pitch = true;
  controller_output.pose.disable_axis.yaw = true;
  controller_output.velocity.disable_axis.x = true;
  controller_output.velocity.disable_axis.y = true;
  controller_output.velocity.disable_axis.z = true;
  controller_output.velocity.disable_axis.roll = true;
  controller_output.velocity.disable_axis.pitch = true;
  controller_output.velocity.disable_axis.yaw = true;

  // Set variables to zero
  controller_output.pose.position.north = 0.0;
  controller_output.pose.position.east = 0.0;
  controller_output.pose.position.depth = 0.0;
  controller_output.pose.orientation.roll = 0.0;
  controller_output.pose.orientation.pitch = 0.0;
  controller_output.pose.orientation.yaw = 0.0;
  controller_output.velocity.linear.x = 0.0;
  controller_output.velocity.linear.y = 0.0;
  controller_output.velocity.linear.z = 0.0;
  controller_output.velocity.angular.x = 0.0;
  controller_output.velocity.angular.y = 0.0;
  controller_output.velocity.angular.z = 0.0;

  // Compute distance to the goal
  const double dist_final = std::sqrt(std::pow(request.final_north - current_state.pose.position.north, 2) +
                                      std::pow(request.final_east - current_state.pose.position.east, 2));

  // Compute desired yaw
  const double desired_yaw = std::atan2(request.final_east - current_state.pose.position.east,
                                        request.final_north - current_state.pose.position.north);

  // Compute desired surge
  const double surge_error = config_.radius - dist_final;
  double desired_surge = -config_.kp * surge_error;
  if (desired_surge > config_.max_surge_velocity)
    desired_surge = config_.max_surge_velocity;
  if (desired_surge < config_.min_surge_velocity)
    desired_surge = config_.min_surge_velocity;
  if (std::fabs(cola2::utils::wrapAngle(desired_yaw - current_state.pose.orientation.yaw)) > 0.5)
    desired_surge = 0.0;

  // Set desired surge
  controller_output.velocity.linear.x = desired_surge;
  controller_output.velocity.disable_axis.x = false;

  // Set desired yaw
  controller_output.pose.orientation.yaw = desired_yaw;
  controller_output.pose.disable_axis.yaw = false;

  // Set desired depth or altitude
  if (request.heave_mode == control::Request::DEPTH)
  {
    controller_output.pose.altitude_mode = false;
    controller_output.pose.altitude = 0.0;
    controller_output.pose.position.depth = request.final_depth;
  }
  else if (request.heave_mode == control::Request::ALTITUDE)
  {
    controller_output.pose.altitude_mode = true;
    controller_output.pose.altitude = request.final_altitude;
    controller_output.pose.position.depth = 0.0;
  }
  else  // BOTH
  {
    if (current_state.pose.altitude <= 0.0)  // No altitude available
    {
      controller_output.pose.altitude_mode = false;
      controller_output.pose.altitude = 0.0;
      controller_output.pose.position.depth = request.final_depth;
    }
    else
    {
      controller_output.pose.altitude_mode = true;
      controller_output.pose.altitude = request.final_altitude;
      controller_output.pose.position.depth = 0.0;
    }
  }
  controller_output.pose.disable_axis.z = false;

  // Set success to false. This mode never ends
  feedback.success = false;

  // Fill additional feedback vars
  feedback.distance_to_end = dist_final;
  feedback.cross_track_error = 0.0;

  // Fill marker
  control::Point initial_point;
  initial_point.x = current_state.pose.position.north;
  initial_point.y = current_state.pose.position.east;
  initial_point.z = current_state.pose.position.depth;
  marker.points_list.push_back(initial_point);
  control::Point final_point;
  final_point.x = request.final_north;
  final_point.y = request.final_east;
  final_point.z = request.final_depth;
  if (controller_output.pose.altitude_mode)
  {
    final_point.z = 0.0;
    if (current_state.pose.altitude > 0.0)
      final_point.z = current_state.pose.position.depth + (current_state.pose.altitude - request.final_altitude);
  }
  marker.points_list.push_back(final_point);
}
