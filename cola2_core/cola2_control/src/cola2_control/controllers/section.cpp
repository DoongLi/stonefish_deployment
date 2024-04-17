/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllers/section.h>
#include <cola2_control/controllers/types.h>

// Constructor
SectionController::SectionController(SectionControllerConfig config) : config_(config)
{
}

void SectionController::setConfig(const SectionControllerConfig& config)
{
  config_ = config;
}

// Compute Method
void SectionController::compute(const control::State& current_state, const control::Request& request,
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

  // Distance to the initial waypoint
  const double dist_initial = std::sqrt(std::pow(request.initial_north - current_state.pose.position.north, 2) +
                                        std::pow(request.initial_east - current_state.pose.position.east, 2));

  // Distance to the final waypoint
  const double dist_final = std::sqrt(std::pow(request.final_north - current_state.pose.position.north, 2) +
                                      std::pow(request.final_east - current_state.pose.position.east, 2));

  // Distance between waypoints
  const double dist_waypoints = std::sqrt(std::pow(request.initial_north - request.final_north, 2) +
                                          std::pow(request.initial_east - request.final_east, 2));

  // Angle of path
  double beta = 0.0;
  if (dist_waypoints > 1e-6)  // Avoid domain error of std::atan2
    beta = std::atan2(request.final_east - request.initial_east,
                      request.final_north - request.initial_north);  // Fossen LOS, Sec. 10.3.2
  const double sin_beta = std::sin(beta);
  const double cos_beta = std::cos(beta);

  // Along-track distance (los_s) and cross-track error (los_e)
  const double los_s = (current_state.pose.position.north - request.initial_north) * cos_beta +
                       (current_state.pose.position.east - request.initial_east) * sin_beta;
  const double los_e = (current_state.pose.position.east - request.initial_east) * cos_beta -
                       (current_state.pose.position.north - request.initial_north) * sin_beta;

  // Compute line-of-sight vector
  double los_x;
  double los_y;
  if (los_s < -config_.delta)  // (los_s <= 0.0)
  {
    // Before initial waypoint. Go straight to the initial waypoint
    los_x = request.initial_north - current_state.pose.position.north;
    los_y = request.initial_east - current_state.pose.position.east;
  }
  else if (los_s >= dist_waypoints)
  {
    // After final waypoint. Go straight to the final waypoint
    los_x = request.final_north - current_state.pose.position.north;
    los_y = request.final_east - current_state.pose.position.east;
  }
  else
  {
    // Orthogonal projection to the path
    const double x_proj = request.initial_north + los_s * cos_beta;
    const double y_proj = request.initial_east + los_s * sin_beta;

    // Compute lookahead distance (los_delta). It is always positive
    double los_delta = 0.0;
    double los_delta_sq = std::pow(config_.delta, 2) - std::pow(los_e, 2);
    if (los_delta_sq > 0.0)
      los_delta = std::sqrt(los_delta_sq);
    if (config_.delta > dist_final)
      los_delta = std::sqrt(std::pow(request.final_north - x_proj, 2) + std::pow(request.final_east - y_proj, 2));

    // Following the section. Use line-of-sight
    los_x = x_proj + los_delta * cos_beta - current_state.pose.position.north;
    los_y = y_proj + los_delta * sin_beta - current_state.pose.position.east;
  }

  // Compute desired surge
  double desired_surge = std::min(request.surge_velocity, config_.max_surge_velocity);
  const double min_surge = std::min(config_.min_surge_velocity, desired_surge);
  const double min_distance = std::min(dist_final, dist_initial);
  if ((min_distance > 0.0) && (min_distance < config_.distance_to_max_velocity))
  {
    const double ratio = min_distance / config_.distance_to_max_velocity;
    desired_surge -= (desired_surge - min_surge) * (1.0 - ratio);  // Decrease speed using min distance to waypoint
  }

  // Compute desired yaw
  double desired_yaw = std::atan2(los_y, los_x);

  // Stop surge when the robot is close in 2D
  if (dist_final < 0.8 * request.tolerance_xy)
  {
    desired_surge = 0.0;
  }

  // Stop yaw when the robot is very close in 2D
  if (dist_final < 0.4 * request.tolerance_xy)
  {
    desired_yaw = current_state.pose.orientation.yaw;
  }

  // Penalize desired surge according to yaw error
  const double yaw_angle_error = cola2::utils::wrapAngle(desired_yaw - current_state.pose.orientation.yaw);
  const double yaw_factor = std::min(std::max(1.0 - (std::fabs(yaw_angle_error) - 0.3) / 1.0, 0.0), 1.0);
  desired_surge *= yaw_factor;

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

  // Set success
  const double dist_heave =
      (controller_output.pose.altitude_mode ? request.final_altitude - current_state.pose.altitude :
                                              request.final_depth - current_state.pose.position.depth);
  feedback.success = (dist_final < request.tolerance_xy) && (std::fabs(dist_heave) < config_.tolerance_z);

  // Fill additional feedback vars
  feedback.distance_to_end = dist_final;
  feedback.cross_track_error = los_e;

  // Fill marker
  control::Point initial_point;
  initial_point.x = request.initial_north;
  initial_point.y = request.initial_east;
  initial_point.z = request.initial_depth;
  marker.points_list.push_back(initial_point);
  control::Point final_point;
  final_point.x = request.final_north;
  final_point.y = request.final_east;
  final_point.z = request.final_depth;
  marker.points_list.push_back(final_point);
}
