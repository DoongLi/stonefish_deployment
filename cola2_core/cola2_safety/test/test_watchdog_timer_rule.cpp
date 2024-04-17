/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/watchdog_timer.h>
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
  diagnostic_status.name = "/safety/watchdog_timer";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "elapsed_time";
  diagnostic_key_value.value = "0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::WatchdogTimer watchdog_timer("watchdog_timer");
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  watchdog_timer.periodicUpdate(ros::Time::now(), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  double timeout = 3600.0;
  ros::param::set("~watchdog_timer/timeout", timeout);
  watchdog_timer.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning
  watchdog_timer.periodicUpdate(ros::Time::now(), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  watchdog_timer.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WatchdogTimer::INIT_TIME + 1.0), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate running fine
  watchdog_timer.diagnosticsUpdate(diagnostic_array);
  watchdog_timer.periodicUpdate(ros::Time::now(), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate no diagnostics
  watchdog_timer.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WatchdogTimer::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "06 ";

  // Simulate no diagnostics after some time
  watchdog_timer.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::WatchdogTimer::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "07 ";

  // Simulate timeout
  diagnostic_array.status[0].values[0].value = std::to_string(timeout + 1);
  watchdog_timer.diagnosticsUpdate(diagnostic_array);
  watchdog_timer.periodicUpdate(ros::Time::now(), &status_code);
  if (watchdog_timer.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "08 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::WatchdogTimer* base_class_ptr = new SafetyRules::WatchdogTimer("watchdog_timer");
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
  ros::init(argc, argv, "test_watchdog_timer_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
