/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/temperature.h>
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
  diagnostic_status.name = "/safety/temperature_pc_cylinder";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "temperature";
  diagnostic_key_value.value = "20.0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::Temperature temperature("temperature_pc_cylinder");
  if (temperature.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  temperature.periodicUpdate(ros::Time::now(), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  double max_temperature = 50.0;
  ros::param::set("~temperature_pc_cylinder/diagnostic_status_name", "/safety/temperature_pc_cylinder");
  ros::param::set("~temperature_pc_cylinder/max_temperature", max_temperature);
  temperature.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning
  temperature.periodicUpdate(ros::Time::now(), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  temperature.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Temperature::INIT_TIME + 1.0), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate running fine
  temperature.diagnosticsUpdate(diagnostic_array);
  temperature.periodicUpdate(ros::Time::now(), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate no diagnostics
  temperature.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Temperature::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "06 ";

  // Simulate no diagnostics after some time
  temperature.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Temperature::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "07 ";

  // Simulate high temperature
  diagnostic_array.status[0].values[0].value = std::to_string(max_temperature + 1.0);
  temperature.diagnosticsUpdate(diagnostic_array);
  temperature.periodicUpdate(ros::Time::now(), &status_code);
  if (temperature.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "08 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Temperature* base_class_ptr = new SafetyRules::Temperature("temperature_pc_cylinder");
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
  ros::init(argc, argv, "test_temperature_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
