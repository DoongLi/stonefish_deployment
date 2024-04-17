/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Transform handler for COLA2.
 */

#ifndef COLA2_NAV_TRANSFORM_HANDLER_H_
#define COLA2_NAV_TRANSFORM_HANDLER_H_

#include <cola2_lib/utils/angles.h>
#include <cola2_lib_ros/this_node.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <map>
#include <string>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup Transforms
 * @{
 */

/**
 * \brief Eigen aligned allocator for transforms.
 */
using MapString2Isometry3d = std::map<std::string, Eigen::Isometry3d, std::less<std::string>,
                                      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>;

/**
 * @brief The TransformHandler class queries and saves transforms with origin at the vehicle frame.
 */
class TransformHandler
{
private:
  MapString2Isometry3d transforms_;         //!< transforms from the robot to the sensors.
  tf2_ros::Buffer tf_buffer_;               //!< transform buffer.
  tf2_ros::TransformListener tf_listener_;  //!< transform listener.
  std::string base_frame_;                  //!< frame used as a base for computing transformations.

  /**
   * @brief Convert from ROS TFs to Eigen::Isometry3d.
   *
   * @param trans ROS Transform.
   * @return Returns Eigen Isometry3d.
   */
  Eigen::Isometry3d tfTransformToEigen(const geometry_msgs::TransformStamped& trans) const;

public:
  /**
   * @brief Constructor that queries the namespace to know vehicle frame.
   */
  TransformHandler();

  /**
   * @brief Constructor that sets the base frame.
   *
   * @param frame Base frame of the vehicle.
   */
  TransformHandler(const std::string& frame);

  /**
   * @brief Get a static transform from the map or query the listener and save.
   *
   * @param frame Frame.
   * @param transform Transform.
   * @return Returns false if can not find the transform.
   */
  [[deprecated("use the version with Eigen::Isometry3d")]] bool getTransform(const std::string& frame,
                                                                             Eigen::Affine3d& transform);
  /**
   * @brief Get a static transform from the map or query the listener and save.
   *
   * @param frame Frame.
   * @param transform Transform.
   * @param time Time of the Transform. ::ros::Time(0) (default) for the latest transform.
   * @return Returns false if can not find the transform.
   */
  bool getTransform(const std::string& frame, Eigen::Isometry3d& transform);

  /**
   * @brief Get a transform by querying the listener.
   *
   * @param frame Frame.
   * @param transform Transform.
   * @param time Time of the Transform. ::ros::Time(0) (default) for the latest transform.
   * @return Returns false if can not find the transform.
   */
  [[deprecated("use the version with Eigen::Isometry3d")]] bool getDynamicTransform(
      const std::string& frame, Eigen::Affine3d& transform, const ::ros::Time& time = ::ros::Time(0)) const;
  /**
   * @brief Get a transform by querying the listener.
   *
   * @param frame Frame.
   * @param transform Transform.
   * @param time Time of the Transform. ::ros::Time(0) (default) for the latest transform.
   * @return Returns false if can not find the transform.
   */
  bool getDynamicTransform(const std::string& frame, Eigen::Isometry3d& transform,
                           const ::ros::Time& time = ::ros::Time(0)) const;

  /**
   * @brief Add transform manually (useful when transform listener cannot listen).
   *
   * @param frame Frame.
   * @param x X offset from vehicle base frame.
   * @param y Y offset from vehicle base frame.
   * @param z Z offset from vehicle base frame.
   * @param qx First value of the quaternion defining orientation wrt vehicle base frame.
   * @param qy Second value of the quaternion defining orientation wrt vehicle base frame.
   * @param qz Third value of the quaternion defining orientation wrt vehicle base frame.
   * @param qw Fourth value of the quaternion defining orientation wrt vehicle base frame.
   */
  void setTransformManually(const std::string& frame, const double x, const double y, const double z, const double qx,
                            const double qy, const double qz, const double qw);
};
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_NAV_TRANSFORM_HANDLER_H_
