/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/this_node.h"
#include <ros/ros.h>

std::string cola2::ros::getNamespace()
{
  std::string ns = ::ros::this_node::getNamespace();

  std::size_t pos = ns.find("//");
  while (pos != std::string::npos)
  {
    ns.replace(pos, 2, "/");
    pos = ns.find("//");
  }
  return ns;
}

std::string cola2::ros::getNamespaceNoInitialDash()
{
  const std::string ns = getNamespace();
  return ns.substr(1, ns.size() - 1);  // skip first character
}

std::string cola2::ros::getUnresolvedNodeName()
{
  std::string node_name = ::ros::this_node::getName();
  std::size_t pos = node_name.find_last_of("/");
  return node_name.substr(pos + 1);
}
