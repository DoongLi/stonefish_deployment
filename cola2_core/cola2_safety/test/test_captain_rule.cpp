/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/captain.h>
#include <cola2_msgs/CaptainStateFeedback.h>
#include <cola2_msgs/KeyValue.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cstdint>
#include <string>

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create publishers
  ros::Publisher pub = nh.advertise<cola2_msgs::CaptainStateFeedback>(cola2::ros::getNamespace() + "/captain/state_feedback", 1, true);

  // Create error codes message
  std::string error_codes;

  // Create captain state feedback message
  cola2_msgs::CaptainStateFeedback csf_msg;
  csf_msg.header.stamp = ros::Time::now();
  csf_msg.state = cola2_msgs::CaptainStateFeedback::ACTIVE;
  cola2_msgs::KeyValue keyvalue;
  keyvalue.key = "current_step";
  keyvalue.value = "1000";
  csf_msg.keyvalues.push_back(keyvalue);

  // Create rule
  SafetyRules::Captain captain("captain", &nh);
  if (captain.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Publish wrong captain state feedback
  pub.publish(csf_msg);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();

  // Check status code
  std::uint32_t status_code = 0;
  captain.periodicUpdate(ros::Time::now(), &status_code);
  if (captain.getLevel() != SafetyRules::SafetyLevel::INFORMATIVE)
    error_codes += "02 ";
  if (status_code != 0)
    error_codes += "03 ";

  // Now publish correct message
  csf_msg.keyvalues[0].value = "23";
  pub.publish(csf_msg);
  for (double now = ros::Time::now().toSec(); ros::Time::now().toSec() - now < 0.5;)
    ros::spinOnce();
  captain.periodicUpdate(ros::Time::now(), &status_code);
  if (captain.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "04 ";
  if (status_code != 23)
    error_codes += "05 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Captain* base_class_ptr = new SafetyRules::Captain("captain", &nh);
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
  ros::init(argc, argv, "test_captain_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
