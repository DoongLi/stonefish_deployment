/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Service Client helper for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_

#include <cola2_lib/utils/thread_safe_flag.h>
#include <ros/ros.h>
#include <functional>
#include <stdexcept>
#include <thread>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup ServiceHelper
 * @{
 */

/**
 * @brief Connect to the specified service name and type.
 *
 * @param nh Node handler that wants to connect to a service.
 * @param srv_name Full service name to connect to.
 * @param wait Time to wait between tries, defaults to 1.0.
 * @param ServiceType Type of service used in the connection.
 * @return A ServiceClient to the specified service.
 */
template <class TSrv>
::ros::ServiceClient connectToService(::ros::NodeHandle& nh, const std::string& srv_name, const double wait = 1.0)
{
  ::ros::ServiceClient service = nh.serviceClient<TSrv>(srv_name);
  while (!::ros::isShuttingDown())
  {
    if (service.waitForExistence(::ros::Duration(wait)))
      break;
    ROS_WARN_STREAM("Waiting for client to service: " << srv_name);
    ::ros::Duration(wait).sleep();
  }
  return service;
}

/**
 * @brief Helper function.
 *
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @param success Success.
 * @param done Done flag.
 */
template <typename TSrv, typename TReq, typename TRes>
void callServiceHelper(TSrv* srv, const TReq* req, TRes* res, bool* success, cola2::utils::ThreadSafeFlag* done)
{
  *success = srv->call(*req, *res);
  done->setState(true);
}

/**
 * @brief This function allows calling a service with a timeout.
 *
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @param sucess Sucess.
 * @param timeout Timeout in seconds.
 * @return It returns true if the service returned on time and false otherwise.
 */
template <typename TSrv, typename TReq, typename TRes>
bool callServiceWithTimeout(TSrv& srv, const TReq& req, TRes& res, bool& success, double timeout)
{
  // Start dedicated thread for the service and wait
  timeout = timeout < 0.0 ? 0.0 : timeout;
  cola2::utils::ThreadSafeFlag* done = new cola2::utils::ThreadSafeFlag(false);  // This may need to remain on memory
  std::function<void()> callback_func =
      std::bind(callServiceHelper<TSrv, TReq, TRes>, &srv, &req, &res, &success, done);
  std::thread th(callback_func);
  done->timedBlockingWaitFor(true, timeout);

  // Check if the service returned on time. Join its thread, free memory and return
  if (done->getState())
  {
    th.join();
    delete done;
    return true;
  }

  // Otherwise, detach thread and leak memory. There is nothing better we can do
  th.detach();
  return false;
}

/**
 * @brief This function allows calling a service with a timeout. It throws if the timeout expires, or if a wait time
 * has been configured and the service is not available after waiting. The template parameter must be provided
 * if the compiler type deduction fails.
 *
 * @param nh Node handle.
 * @tparam req Service request.
 * @tparam res Service response.
 * @param srv_name Service name.
 * @param timeout Timeout in seconds.
 * @param wait_time Time in seconds to wait for the service if it does not exist.
 * @return It returns true if the service returned true and false otherwise.
 */
template <typename TSrv>
bool callServiceWithTimeout(::ros::NodeHandle& nh, typename TSrv::Request& req, typename TSrv::Response& res,
                            const std::string& srv_name, const double timeout, const double wait_time = 0.0)
{
  // Create service client
  ::ros::ServiceClient* srv_client_ptr = new ::ros::ServiceClient(nh.serviceClient<TSrv>(srv_name));

  // Check if a wait time has been given
  if ((wait_time > 0.0) && (!srv_client_ptr->waitForExistence(::ros::Duration(wait_time))))
  {
    delete srv_client_ptr;
    throw std::runtime_error(std::string("Wait time exceeded for service ") + srv_name);
  }

  // Create a copy of the request and the response, create a success bool and create thread safe flag
  typename TSrv::Request* req_ptr = new typename TSrv::Request();
  *req_ptr = req;
  typename TSrv::Response* res_ptr = new typename TSrv::Response();
  *res_ptr = res;
  bool* success_ptr = new bool();
  cola2::utils::ThreadSafeFlag* done_ptr = new cola2::utils::ThreadSafeFlag(false);

  // Start thread calling lambda with captures to the pointers
  std::thread th([srv_client_ptr, req_ptr, res_ptr, success_ptr, done_ptr]() {
    *success_ptr = srv_client_ptr->call(*req_ptr, *res_ptr);
    done_ptr->setState(true);
  });

  // Wait for result
  done_ptr->timedBlockingWaitFor(true, (timeout < 0.0 ? 0.0 : timeout));

  // Check if the service failed to return on time
  if (!done_ptr->getState())
  {
    th.detach();
    throw std::runtime_error(std::string("Timeout exceeded for service ") + srv_name + std::string(". Leaking memory"));
  }

  // The call returned on time. Copy back the result
  req = *req_ptr;
  res = *res_ptr;
  bool success = *success_ptr;

  // Join thread and clear memory
  th.join();
  delete done_ptr;
  delete success_ptr;
  delete res_ptr;
  delete req_ptr;
  delete srv_client_ptr;

  // Return what the service has returned
  return success;
}
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_
