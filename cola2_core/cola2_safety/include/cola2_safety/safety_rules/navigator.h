/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_NAVIGATOR_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_NAVIGATOR_H_

#include <cola2_safety/safety_rules/common.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/WorldWaypointReq.h>

namespace SafetyRules
{
class Navigator : public SafetyRuleBaseClass
{
 protected:
  ros::Subscriber sub_nav_, sub_wwr_;
  double last_altitude_wwr_;
  bool filter_init_;
  double last_nav_data_;
  double last_imu_data_;
  double last_depth_data_;
  double last_altitude_data_;
  double last_dvl_data_;
  double last_gps_data_;
  double last_good_data_;
  double nav_data_timeout_;
  double imu_data_timeout_;
  double depth_data_timeout_;
  double altitude_data_timeout_;
  double dvl_data_timeout_;
  double gps_data_timeout_;
  double min_frequency_;
  double last_surfaced_;
  double first_no_altitude_;

  // Methods
  void navCallback(const cola2_msgs::NavSts&);
  void wwrCallback(const cola2_msgs::WorldWaypointReq&);

 public:
  static constexpr double SURFACE_DEPTH = 0.5;
  static constexpr double GPS_SURFACE_TIME = 60.0;

  explicit Navigator(const std::string&, ros::NodeHandle*);
  void parseDiagnostics();
  void periodicUpdate(const ros::Time&, std::uint32_t*);
  bool loadConfigFromParamServer();
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_NAVIGATOR_H_
