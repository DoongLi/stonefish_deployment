/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/teleoperation.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <algorithm>
#include <cstdint>
#include <string>

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create publisher
  ros::Publisher pub = nh.advertise<cola2_msgs::CaptainStatus>(cola2::ros::getNamespace() + "/captain/captain_status", 1, true);

  // Create error codes message
  std::string error_codes;

  // Create fake diagnostic msg
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic_status;
  diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic_status.name = "/control/teleoperation";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "enabled";
  diagnostic_key_value.value = "true";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "last_ack";
  diagnostic_key_value.value = "0.0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::Teleoperation teleoperation("teleoperation", &nh);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  teleoperation.periodicUpdate(ros::Time::now(), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  double timeout = SafetyRules::Teleoperation::INIT_TIME + 10.0;
  ros::param::set("~teleoperation/teleoperation_link_timeout", timeout);
  if (!teleoperation.loadConfigFromParamServer())
  {
    error_codes += "03 ";
  }

  // Simulate no diagnostics at the beginning
  teleoperation.periodicUpdate(ros::Time::now(), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "04 ";

  // Simulate no diagnostics after some time at the beginning
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::INIT_TIME + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "05 ";

  // Simulate running fine
  teleoperation.diagnosticsUpdate(diagnostic_array);
  teleoperation.periodicUpdate(ros::Time::now(), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "06 ";

  // Simulate no diagnostics
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "07 ";

  // Simulate no diagnostics after some time
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::NO_DIAGNOSTICS_ESCALATED_TIME + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "08 ";

  // Simulate captain status not being published
  diagnostic_array.status[0].values[1].value = "0.0";
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::NO_DIAGNOSTICS_TIME + 1.0);
  teleoperation.diagnosticsUpdate(diagnostic_array);
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::NO_DIAGNOSTICS_TIME + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "09 ";

  // Simulate teleoperation timeout
  cola2_msgs::CaptainStatus captain_status;
  captain_status.header.stamp = ros::Time::now();
  captain_status.state = cola2_msgs::CaptainStatus::GOTO;
  pub.publish(captain_status);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  captain_status.header.stamp = ros::Time::now() + ros::Duration(timeout + 1.0);
  captain_status.state = cola2_msgs::CaptainStatus::IDLE;
  pub.publish(captain_status);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  diagnostic_array.status[0].values[1].value = std::to_string(timeout + 1.0);
  diagnostic_array.header.stamp = ros::Time::now() + ros::Duration(timeout + 1.0);
  teleoperation.diagnosticsUpdate(diagnostic_array);
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(timeout + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "10 ";

  // Simulate teleoperation timeout in autonomous mode
  captain_status.header.stamp = ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::NO_DIAGNOSTICS_TIME + 1.0);
  captain_status.state = cola2_msgs::CaptainStatus::GOTO;
  pub.publish(captain_status);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  teleoperation.periodicUpdate(ros::Time::now() + ros::Duration(SafetyRules::Teleoperation::INIT_TIME + 1.0), &status_code);
  if (teleoperation.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "11 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Teleoperation* base_class_ptr = new SafetyRules::Teleoperation("teleoperation", &nh);
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
  ros::init(argc, argv, "test_teleoperation_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
