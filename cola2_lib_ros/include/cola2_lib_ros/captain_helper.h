
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Captain helper for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_

#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/GoalDescriptor.h>
#include <cola2_msgs/Goto.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <functional>
#include <mutex>

namespace cola2
{
namespace rosutils
{
/**
 * @addtogroup CaptainHelper
 * @{
 */

/**
 * @brief Helper function. Not intended to be used on its own.
 *
 * @param is_idle Is captain status idle?
 * @param is_safety Is captain status in safety keep position?
 * @param mtx Mutex.
 * @param first_call Is the first call?
 * @param captain_status Captain Status.
 */
void waitForIdleHelper(bool* is_idle, bool* is_safety, std::mutex* mtx, bool* first_call,
                       const cola2_msgs::CaptainStatus::ConstPtr& captain_status);

/**
 * @brief This function blocks until the captain is in Idle state.
 *
 * @param nh Node handle.
 * @return Returns true if it returned back to Idle and false if it switched to something else.
 */
bool waitForIdle(ros::NodeHandle& nh);

/**
 * @brief  This function calls a service from the captain, such as a goto, mission... and then blocks until the captain
 * is back to idle.
 *
 * @param nh Node handle.
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @return Returns true if the call and the waiting was succesful, and false otherwise.
 */
template <typename TSrv, typename TReq, typename TRes>
bool callServiceAndWaitForIdle(ros::NodeHandle& nh, TSrv& srv, const TReq& req, TRes& res)
{
  bool srv_return = srv.call(req, res);
  if (res.success && srv_return)
  {
    if (waitForIdle(nh))
    {
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Captain switched to safety state instead of idle");
      return false;
    }
  }
  ROS_ERROR_STREAM("Error processing request in callServiceAndWaitForIdle(). Req:" << std::endl
                                                                                   << req << std::endl
                                                                                   << "Res:" << std::endl
                                                                                   << res);
  return false;
}
/** @} */
}  // namespace rosutils
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_
