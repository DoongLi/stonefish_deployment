/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/water_inside.h>
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
  diagnostic_status.name = "/safety/water_inside_pc_cylinder";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "water";
  diagnostic_key_value.value = "false";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::WaterInside water_inside("water_inside_pc_cylinder");
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Simulate no diagnostics at the beginning
  std::uint32_t status_code = 0;
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "02 ";

  // Simulate no diagnostics after some time at the beginning
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WaterInside::INIT_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "03 ";

  // Simulate running fine
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "04 ";

  // Simulate no diagnostics
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WaterInside::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "05 ";

  // Simulate no diagnostics after some time
  water_inside.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WaterInside::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "06 ";

  // Simulate water inside
  diagnostic_array.status[0].values[0].value = "true";
  water_inside.diagnosticsUpdate(diagnostic_array);
  water_inside.periodicUpdate(ros::Time::now(), &status_code);
  if (water_inside.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "07 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::WaterInside* base_class_ptr = new SafetyRules::WaterInside("water_inside_pc_cylinder");
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
