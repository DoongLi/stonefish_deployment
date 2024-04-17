/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Setpoint selector for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROS_SETPOINTS_SELECTOR_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROS_SETPOINTS_SELECTOR_H_

#include <cola2_msgs/Setpoints.h>
#include <ros/ros.h>
#include <string>
#include <utility>
#include <vector>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup SetpointsSelector
 * @{
 */

/**
 * @brief The setpoints selector class receives thruster setpoints and prioritizes safety and open loop commands.
 */
class SetpointsSelector
{
protected:
  static constexpr double EXPIRY_TIME = 1.0;                    //<! Expiry time.
  ::ros::Publisher pub_ts_;                                     //<! Setpoints selector publisher.
  std::vector<std::tuple<std::string, double>> priority_list_;  //<! Priority list.

public:
  /**
   * @brief Constructor.
   *
   * @param nh Node handle reference.
   */
  SetpointsSelector(::ros::NodeHandle& nh);

  /**
   * @brief This method returns true if the setpoint is accepted and false otherwise.
   *
   * @param msg Setpoints message.
   * @return Returns true if the setpoint is accepted and false otherwise.
   */
  bool acceptSetpoints(const cola2_msgs::Setpoints& msg);
};
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROS_SETPOINTS_SELECTOR_H_
