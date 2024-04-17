/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/battery_control_board.h>
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
  diagnostic_status.name = "/safety/battery_control_board";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "status_indicator";
  diagnostic_key_value.value = "0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::BatteryControlBoard battery_control_board("battery_control_board");
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Simulate no diagnostics at the beginning
  battery_control_board.loadConfigFromParamServer();
  std::uint32_t status_code = 0;
  battery_control_board.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "02 ";

  // Simulate no diagnostics after some time at the beginning
  battery_control_board.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryControlBoard::INIT_TIME + 1.0), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "03 ";

  // Simulate running fine
  battery_control_board.diagnosticsUpdate(diagnostic_array);
  battery_control_board.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "04 ";

  // Simulate magnetic switch
  diagnostic_array.status[0].values[0].value = "1";
  battery_control_board.diagnosticsUpdate(diagnostic_array);
  battery_control_board.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "05 ";

  // Simulate BMS error
  diagnostic_array.status[0].values[0].value = "2";
  battery_control_board.diagnosticsUpdate(diagnostic_array);
  battery_control_board.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "06 ";

  // Simulate low battery
  diagnostic_array.status[0].values[0].value = "3";
  battery_control_board.diagnosticsUpdate(diagnostic_array);
  battery_control_board.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "07 ";

  // Simulate no diagnostics
  diagnostic_array.status[0].values[0].value = "0";
  battery_control_board.diagnosticsUpdate(diagnostic_array);
  battery_control_board.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryControlBoard::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "08 ";

  // Simulate no diagnostics after some time
  battery_control_board.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryControlBoard::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (battery_control_board.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "09 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::BatteryControlBoard* base_class_ptr = new SafetyRules::BatteryControlBoard("battery_control_board");
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
  ros::init(argc, argv, "test_battery_control_board_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
