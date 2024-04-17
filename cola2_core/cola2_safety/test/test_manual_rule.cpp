/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_msgs/RecoveryAction.h>
#include <cola2_lib/utils/thread_safe_flag.h>
#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/manual.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cstdint>
#include <string>
#include <thread>

void
callServiceWithErrorLevel(ros::NodeHandle* nh_ptr, const std::uint16_t& level)
{
  cola2::utils::ThreadSafeFlag done(false);
  std::thread th([nh_ptr, level, &done]()
  {
    ros::ServiceClient client(nh_ptr->serviceClient<cola2_msgs::Recovery>("manual/set_level"));
    client.waitForExistence(ros::Duration(1.0));
    cola2_msgs::Recovery::Request req;
    req.requested_action.error_level = level;
    cola2_msgs::Recovery::Response res;
    client.call(req, res);
    done.setState(true);
  });
  while (!done.getState())
    ros::spinOnce();
  th.join();
}

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create error codes message
  std::string error_codes;

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Manual* base_class_ptr = new SafetyRules::Manual("manual", &nh);
  delete base_class_ptr;

  // Create rule
  SafetyRules::Manual manual("manual", &nh);
  if (manual.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Check levels
  std::uint32_t status_code = 0;
  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::NONE);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "02 ";

  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::INFORMATIVE);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "03 ";

  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::ABORT);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::ABORT)
    error_codes += "04 ";

  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::ABORT_AND_SURFACE);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "05 ";

  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::EMERGENCY_SURFACE);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::EMERGENCY_SURFACE)
    error_codes += "06 ";

  callServiceWithErrorLevel(&nh, cola2_msgs::RecoveryAction::DROP_WEIGHT);
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::DROP_WEIGHT)
    error_codes += "07 ";

  callServiceWithErrorLevel(&nh, 100);  // Unknown level
  manual.periodicUpdate(ros::Time::now(), &status_code);
  if (manual.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "08 ";

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
  ros::init(argc, argv, "test_manual_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
