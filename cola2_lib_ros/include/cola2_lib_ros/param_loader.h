
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Param loader for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_

#include <ros/ros.h>
#include <string>
#include <vector>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup ParamLoader
 * @{
 */

/**
 * @brief Reads the parameter param_name from the ROS param server into param_var variable.
 * If the parameter is not found a warning is displayed.
 *
 * @param param_name Parameter name.
 * @tparam param_var Variable where the parameter is saved.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename T>
bool getParam(const std::string param_name, T& param_var)
{
  // Display a message if a parameter is not found in the param server
  if (!::ros::param::getCached(param_name, param_var))
  {
    ROS_WARN_STREAM("Value for parameter " << param_name << " not found in param server!");
    return false;
  }
  return true;
}

/**
 * @brief Reads the parameter param_name from the ROS param server into param_var variable.
 * If not found in param server, outputs a warning and sets the parameter to default_value.
 *
 * @param param_name Parameter name.
 * @tparam param_var Variable where the parameter is saved.
 * @tparam default_value Default value if parameter is not found in ROS param server.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename T>
bool getParam(const std::string param_name, T& param_var, T default_value)
{
  // Display a message if a parameter is not found in the param server
  if (!::ros::param::getCached(param_name, param_var))
  {
    ROS_WARN_STREAM("Value for parameter " << param_name << " not found in param server! Using default value "
                                           << default_value);
    param_var = default_value;
    return false;
  }
  return true;
}

/**
 * @brief Reads the vector parameter param_name from the ROS param server into data vector.
 * If not found in param server, outputs a fatal message.
 *
 * @param param_name Parameter name.
 * @tparam data Vector wheter the parameter values are saved.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename ParamType>
bool getParamVector(const std::string param_name, std::vector<ParamType>& data)
{
  // Clear vector
  data.clear();
  // Take the param vector and copy it to a std::vector<ParamType>
  XmlRpc::XmlRpcValue my_list;
  if (::ros::param::getCached(param_name, my_list))
  {
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      data.push_back(static_cast<ParamType>(my_list[i]));
    }
  }
  else
  {
    ROS_FATAL_STREAM("Invalid parameters for " << param_name << " in param server!");
    return false;
  }
  return true;
}
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_
