/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/virtual_cage.h>
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
  diagnostic_status.name = "/safety/virtual_cage";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "enabled";
  diagnostic_key_value.value = "true";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "inside_virtual_cage";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::VirtualCage virtual_cage("virtual_cage");
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  virtual_cage.periodicUpdate(ros::Time::now(), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  ros::param::set("~virtual_cage/enables_abort_and_surface", false);
  virtual_cage.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning
  virtual_cage.periodicUpdate(ros::Time::now(), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  // Simulate no diagnostics after some time at the beginning
  virtual_cage.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::VirtualCage::INIT_TIME + 1.0), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate running fine
  virtual_cage.diagnosticsUpdate(diagnostic_array);
  virtual_cage.periodicUpdate(ros::Time::now(), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "05 ";

  // Simulate no diagnostics
  virtual_cage.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::VirtualCage::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "06 ";

  // Simulate no diagnostics after some time
  virtual_cage.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::VirtualCage::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "07 ";

  // Simulate out of virtual cage
  diagnostic_array.status[0].values[1].value = "false";
  virtual_cage.diagnosticsUpdate(diagnostic_array);
  virtual_cage.periodicUpdate(ros::Time::now(), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "08 ";

  // And now, out of virtual cage but with abort and surface
  ros::param::set("~virtual_cage/enables_abort_and_surface", true);
  virtual_cage.loadConfigFromParamServer();
  virtual_cage.periodicUpdate(ros::Time::now(), &status_code);
  if (virtual_cage.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "09 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::VirtualCage* base_class_ptr = new SafetyRules::VirtualCage("virtual_cage");
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
  ros::init(argc, argv, "test_virtual_cage_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
