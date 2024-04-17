/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/combined_water_inside.h>
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
  diagnostic_status.name = "/safety/extra_housing";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "water_inside";
  diagnostic_key_value.value = "false";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);
  diagnostic_status.name = "/safety/main_control_board";
  diagnostic_status.values[0].key = "water_internal_inside";
  diagnostic_key_value.key = "water_external_inside";
  diagnostic_key_value.value = "false";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);
  diagnostic_status.name = "/safety/battery_control_board";
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::CombinedWaterInside water_inside("combined_water_inside");
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config, but only checking main housing
  ros::param::set("~combined_water_inside/check_battery_housing", false);
  ros::param::set("~combined_water_inside/check_extra_housing", true);  // No effect if check_battery_housing is false
  water_inside.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::CombinedWaterInside::INIT_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate running fine
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate no diagnostics
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::CombinedWaterInside::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "06 ";

  // Simulate no diagnostics after some time
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::CombinedWaterInside::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "07 ";

  // Check water inside in only the main housing (through battery board)
  diagnostic_array.status[2].values[0].value = "true";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "08 ";

  // From now on, all housings will be checked
  ros::param::set("~combined_water_inside/check_battery_housing", true);
  water_inside.loadConfigFromParamServer();

  // Check water inside in only the main housing, but wrong redundancy
  diagnostic_array.status[1].values[0].value = "true";
  diagnostic_array.status[2].values[0].value = "false";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "09 ";

  // Check water inside in only the main housing
  diagnostic_array.status[2].values[1].value = "true";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "10 ";

  // Check water inside main and batteries housings
  diagnostic_array.status[2].values[0].value = "true";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "11 ";

  // Check water inside main, batteries and extra housings
  diagnostic_array.status[0].values[0].value = "true";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "12 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::CombinedWaterInside* base_class_ptr = new SafetyRules::CombinedWaterInside("combined_water_inside");
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
  ros::init(argc, argv, "test_water_inside_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
