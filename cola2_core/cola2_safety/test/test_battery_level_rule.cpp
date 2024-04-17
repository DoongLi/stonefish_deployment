/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/battery_level.h>
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
  diagnostic_status.name = "/safety/battery_level";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "charge";
  diagnostic_key_value.value = "40";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "voltage";
  diagnostic_key_value.value = "40";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::BatteryLevel battery_level("battery_level");
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  double min_charge = 25.0;
  double min_voltage = 25.0;
  ros::param::set("~battery_level/min_charge", min_charge);
  ros::param::set("~battery_level/min_voltage", min_voltage);
  battery_level.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  battery_level.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryLevel::INIT_TIME + 1.0), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate running fine
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate no diagnostics
  battery_level.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryLevel::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "06 ";

  // Simulate no diagnostics after some time
  battery_level.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::BatteryLevel::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "07 ";

  // Simulate low charge
  diagnostic_array.status[0].values[0].value = std::to_string(min_charge * (0.5 + 0.5 * SafetyRules::BatteryLevel::LOW_CHARGE_FACTOR));
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "08 ";

  // Simulate low charge and low voltage
  diagnostic_array.status[0].values[1].value = std::to_string(min_voltage + 0.5 * SafetyRules::BatteryLevel::LOW_VOLTAGE_OFFSET);
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "09 ";

  // Simulate low voltage
  diagnostic_array.status[0].values[0].value = std::to_string(min_charge * (1.5 * SafetyRules::BatteryLevel::LOW_CHARGE_FACTOR - 0.5));
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "10 ";

  // Simulate charge below threshold
  diagnostic_array.status[0].values[0].value = std::to_string(min_charge - 1.0);
  diagnostic_array.status[0].values[1].value = std::to_string(min_voltage + SafetyRules::BatteryLevel::LOW_VOLTAGE_OFFSET + 1.0);
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "11 ";

  // Simulate charge and voltage below threshold
  diagnostic_array.status[0].values[1].value = std::to_string(min_voltage - 1.0);
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "12 ";

  // Simulate voltage below threshold
  diagnostic_array.status[0].values[0].value = std::to_string(min_charge * 1.6);
  battery_level.diagnosticsUpdate(diagnostic_array);
  battery_level.periodicUpdate(ros::Time::now(), &status_code);
  if (battery_level.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "13 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::BatteryLevel* base_class_ptr = new SafetyRules::BatteryLevel("battery_level");
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
  ros::init(argc, argv, "test_battery_level_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
