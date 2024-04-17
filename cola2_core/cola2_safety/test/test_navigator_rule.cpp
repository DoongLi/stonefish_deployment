/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/navigator.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstdint>
#include <string>

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create publishers
  ros::Publisher pub_nav =
      nh.advertise<cola2_msgs::NavSts>(cola2::ros::getNamespace() + "/navigator/navigation", 10, true);
  ros::Publisher pub_wwr = nh.advertise<cola2_msgs::WorldWaypointReq>(cola2::ros::getNamespace() + "/controller/"
                                                                                                   "world_waypoint_req",
                                                                      10, true);

  // Create error codes message
  std::string error_codes;

  // Create rule
  SafetyRules::Navigator navigator("navigator", &nh);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  const double timeout = SafetyRules::Navigator::INIT_TIME - 5.0;  // Must be smaller than init time
  const double min_frequency = 30.0;
  ros::param::set("~navigator/nav_data_timeout", timeout);
  ros::param::set("~navigator/imu_data_timeout", timeout);
  ros::param::set("~navigator/depth_data_timeout", timeout);
  ros::param::set("~navigator/altitude_data_timeout", timeout);
  ros::param::set("~navigator/dvl_data_timeout", timeout);
  ros::param::set("~navigator/gps_data_timeout", timeout);
  ros::param::set("~navigator/min_frequency", min_frequency);
  navigator.loadConfigFromParamServer();

  // Create fake diagnostic msg
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic_status;
  diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic_status.name = "/navigation/navigator";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "filter_init";
  diagnostic_key_value.value = "true";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_imu_data";
  diagnostic_key_value.value = "0.0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_depth_data";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_altitude_data";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_dvl_data";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_gps_data";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "frequency";
  diagnostic_key_value.value = std::to_string(min_frequency + 1.0);
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Simulate no diagnostics at the beginning
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Publish initial navigation message at surface
  cola2_msgs::NavSts nav_sts;
  nav_sts.header.stamp = ros::Time::now();
  nav_sts.position.depth = 0.0;
  pub_nav.publish(nav_sts);
  nav_sts.global_position.latitude = 0.1;
  pub_nav.publish(nav_sts);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();

  // Publish initial world waypoint request message with altitude mode enabled
  cola2_msgs::WorldWaypointReq wwr;
  wwr.header.stamp = ros::Time::now();
  wwr.altitude_mode = true;
  pub_wwr.publish(wwr);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();

  // Simulate running fine
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate low frequency
  diagnostic_array.status[0].values[6].value = std::to_string(min_frequency - 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "06 ";
  diagnostic_array.status[0].values[6].value = std::to_string(min_frequency + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);

  // Simulate no navigation
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(timeout + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "07 ";

  // Simulate no navigation escalated
  navigator.periodicUpdate(
      ros::Time::now() + ros::Duration(std::max(timeout + 1.0, SafetyRules::Navigator::INIT_TIME + 1.0)), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "08 ";

  // Simulate no diagnostics
  nav_sts.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::NO_DIAGNOSTICS_TIME + 1.0);
  pub_nav.publish(nav_sts);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::NO_DIAGNOSTICS_TIME + 1.0),
                           &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "09 ";

  // Simulate no diagnostics after some time
  navigator.periodicUpdate(
      ros::Time::now() + ros::Duration(SafetyRules::Navigator::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "10 ";

  // Check EKF init
  diagnostic_array.status[0].values[0].value = "false";
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "11 ";

  // Check imu data age
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_array.status[0].values[0].value = "true";
  diagnostic_array.status[0].values[1].value = std::to_string(timeout + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "12 ";
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "13 ";

  // Check depth data age
  diagnostic_array.status[0].values[1].value = "0.0";
  diagnostic_array.status[0].values[2].value = std::to_string(timeout + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "14 ";
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "15 ";

  // Check altitude data age
  diagnostic_array.status[0].values[2].value = "0.0";
  diagnostic_array.status[0].values[3].value = std::to_string(timeout + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "16 ";
  wwr.header.stamp += ros::Duration(4.0);
  pub_wwr.publish(wwr);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(4.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "17 ";

  // Check dvl data age
  diagnostic_array.status[0].values[3].value = "0.0";
  diagnostic_array.status[0].values[4].value = std::to_string(timeout + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now(), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "18 ";
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0);
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::INIT_TIME + 1.0), &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "19 ";

  // Check gps data age at surface
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::GPS_SURFACE_TIME + 1.0);
  diagnostic_array.status[0].values[4].value = "0.0";
  diagnostic_array.status[0].values[5].value = std::to_string(timeout + 1.0);
  nav_sts.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Navigator::GPS_SURFACE_TIME + 1.0);
  pub_nav.publish(nav_sts);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  navigator.diagnosticsUpdate(diagnostic_array);
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::GPS_SURFACE_TIME + 1.0),
                           &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "20 ";

  // Check gps data age submerged
  nav_sts.position.depth = SafetyRules::Navigator::SURFACE_DEPTH + 1.0;
  pub_nav.publish(nav_sts);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  navigator.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Navigator::GPS_SURFACE_TIME + 1.0),
                           &status_code);
  if (navigator.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "21 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Navigator* base_class_ptr = new SafetyRules::Navigator("navigator", &nh);
  delete base_class_ptr;

  if (!error_codes.empty())
  {
    ROS_FATAL_STREAM("Test failed with error codes: " << error_codes);
    FAIL();
  }
}

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
    ros::console::notifyLoggerLevelsChanged();
  ros::init(argc, argv, "test_navigator_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
