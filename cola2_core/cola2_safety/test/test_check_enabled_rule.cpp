/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/check_enabled.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cstdint>
#include <string>

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create error codes message
  std::string error_codes;

  // Create fake diagnostic msg
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic_status;
  diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic_status.name = "/control/controller";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "enabled";
  diagnostic_key_value.value = "false";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);
  diagnostic_status.name = "/control/thrusters";
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::CheckEnabled check_enabled("check_enabled");
  if (check_enabled.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  check_enabled.periodicUpdate(ros::Time::now(), &status_code);
  if (check_enabled.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  std::vector<std::string> diagnostics_to_check = {diagnostic_array.status[0].name, diagnostic_array.status[1].name};
  ros::param::set("~check_enabled/diagnostic_status_names_to_check", diagnostics_to_check);
  ros::param::set("~check_enabled/level_if_not_enabled", 100);
  check_enabled.loadConfigFromParamServer();
  ros::param::set("~check_enabled/level_if_not_enabled", static_cast<int>(SafetyRules::SafetyLevel::EMERGENCY_SURFACE));
  check_enabled.loadConfigFromParamServer();

  // Simulate incorrect diagnostics
  check_enabled.periodicUpdate(ros::Time::now(), &status_code);
  if (check_enabled.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate incorrect diagnostics after init time
  check_enabled.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::CheckEnabled::INIT_TIME + 1.0), &status_code);
  if (check_enabled.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "04 ";

  // Simulate correct diagnostics
  diagnostic_array.status[0].values[0].value = "true";
  diagnostic_array.status[1].values[0].value = "true";
  check_enabled.diagnosticsUpdate(diagnostic_array);
  check_enabled.periodicUpdate(ros::Time::now(), &status_code);
  if (check_enabled.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::CheckEnabled* base_class_ptr = new SafetyRules::CheckEnabled("check_enabled");
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
  ros::init(argc, argv, "test_check_enabled_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
