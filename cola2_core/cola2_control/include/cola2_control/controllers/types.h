
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_TYPES_H_
#define COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_TYPES_H_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace control
{
// Default values
const double d_double(std::numeric_limits<double>::quiet_NaN());
const std::uint64_t d_uint(std::numeric_limits<std::uint64_t>::quiet_NaN());
const bool d_bool(false);
const std::string d_string("Not init");

class Point
{
 public:
  double x;
  double y;
  double z;
  Point() : x(d_double), y(d_double), z(d_double)
  {
  }
};

class Vector6d
{
 public:
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  Vector6d() : x(d_double), y(d_double), z(d_double), roll(d_double), pitch(d_double), yaw(d_double)
  {
  }
};

class Nav
{
 public:
  double ned_origin_latitude;
  double ned_origin_longitude;
  double north;
  double east;
  double depth;
  double roll;
  double pitch;
  double yaw;
  double altitude;
  Nav() : ned_origin_latitude(d_double), ned_origin_longitude(d_double), north(d_double), east(d_double),
    depth(d_double), roll(d_double), pitch(d_double), yaw(d_double), altitude(d_double)
  {
  }
};

class RPY
{
 public:
  double roll;
  double pitch;
  double yaw;
  RPY() : roll(d_double), pitch(d_double), yaw(d_double)
  {
  }
};

class NED
{
 public:
  double ned_origin_latitude;
  double ned_origin_longitude;
  double north;
  double east;
  double depth;
  NED() : ned_origin_latitude(d_double), ned_origin_longitude(d_double), north(d_double),
    east(d_double), depth(d_double)
  {
  }
};

struct PointsList
{
  std::vector<control::Point> points_list;
};

class Request
{
 public:
  double ned_origin_latitude;
  double ned_origin_longitude;
  double initial_north;
  double initial_east;
  double initial_depth;
  double final_north;
  double final_east;
  double final_depth;
  double final_yaw;
  double final_altitude;
  std::uint64_t heave_mode;
  static const std::uint64_t DEPTH = 0;
  static const std::uint64_t ALTITUDE = 1;
  static const std::uint64_t BOTH = 2;
  double surge_velocity;
  double tolerance_xy;
  double timeout;
  std::uint64_t controller_type;
  static const std::uint64_t SECTION = 0;
  static const std::uint64_t ANCHOR = 1;
  static const std::uint64_t HOLONOMIC_KEEP_POSITION = 2;
  std::string requester;
  std::uint64_t priority;
  Request() : ned_origin_latitude(d_double), ned_origin_longitude(d_double), initial_north(d_double),
    initial_east(d_double), initial_depth(d_double), final_north(d_double), final_east(d_double),
    final_depth(d_double), final_yaw(d_double), final_altitude(d_double), heave_mode(d_uint),
    surge_velocity(d_double), tolerance_xy(d_double), timeout(d_double), controller_type(d_uint),
    requester(d_string), priority(d_uint)
  {
  }
};

class Pose
{
 public:
  control::NED position;
  control::RPY orientation;
  control::Vector6d disable_axis;
  double altitude;
  bool altitude_mode;
  Pose() : altitude(d_double), altitude_mode(d_bool)
  {
  }
};

struct Velocity
{
  control::Point linear;
  control::Point angular;
  control::Vector6d disable_axis;
};

struct State
{
  control::Pose pose;
  control::Velocity velocity;
};

class Feedback
{
 public:
  double cross_track_error;
  double distance_to_end;
  bool success;
  Feedback()
    : cross_track_error(d_double)
    , distance_to_end(d_double)
    , success(d_bool)
  {
  }
};
}  // namespace control

#endif  // COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_TYPES_H_
