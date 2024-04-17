/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/transform_handler.h"

cola2::ros::TransformHandler::TransformHandler() : tf_buffer_(), tf_listener_(tf_buffer_)
{
  base_frame_ = cola2::ros::getNamespaceNoInitialDash() + std::string("/base_link");
}

cola2::ros::TransformHandler::TransformHandler(const std::string& frame) : tf_buffer_(), tf_listener_(tf_buffer_)
{
  base_frame_ = frame;
}

bool cola2::ros::TransformHandler::getTransform(const std::string& frame, Eigen::Isometry3d& transform)
{
  // Cleanup / at begining of frame (workaround, should not happen)
  std::string cleaned_frame = frame;
  if (frame[0] == '/')
  {
    cleaned_frame = frame.substr(1, frame.size() - 1);
  }

  // Return identity when frame is vehicle frame
  if (cleaned_frame.compare(base_frame_) == 0)
  {
    transform = Eigen::Isometry3d::Identity();
    return true;
  }
  // Look for the transform
  try
  {
    // Found in map
    transform = transforms_.at(cleaned_frame);
    return true;
  }
  catch (const std::out_of_range&)
  {
    // Need to query it
    if (getDynamicTransform(cleaned_frame, transform))
    {
      transforms_[cleaned_frame] = transform;
      ROS_INFO_STREAM("Transform Handler added '" << cleaned_frame);
      ROS_INFO_STREAM("trans: " << transform.translation().transpose());
      ROS_INFO_STREAM("rpy:   " << cola2::utils::rotation2euler(transform.rotation()).transpose());
      return true;
    }
  }
  return false;
}

bool cola2::ros::TransformHandler::getDynamicTransform(const std::string& frame, Eigen::Isometry3d& transform,
                                                       const ::ros::Time& time) const
{
  // Return identity when frame is vehicle frame
  if (frame.compare(base_frame_) == 0)
  {
    transform = Eigen::Isometry3d::Identity();
    return true;
  }
  // Look for the transform
  try
  {
    const geometry_msgs::TransformStamped tf_trans = tf_buffer_.lookupTransform(base_frame_, frame, time);
    transform = tfTransformToEigen(tf_trans);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_FATAL_STREAM("Unable to get dynamic transform: " << frame << "\n  - " << std::string(ex.what()));
  }
  return false;
}

Eigen::Isometry3d cola2::ros::TransformHandler::tfTransformToEigen(const geometry_msgs::TransformStamped& trans) const
{
  const Eigen::Quaterniond quat(trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y,
                                trans.transform.rotation.z);
  Eigen::Isometry3d affine(quat.toRotationMatrix());
  affine.translation() =
      Eigen::Vector3d(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
  return affine;
}

void cola2::ros::TransformHandler::setTransformManually(const std::string& frame, const double x, const double y,
                                                        const double z, const double qx, const double qy,
                                                        const double qz, const double qw)
{
  Eigen::Quaterniond quat(qw, qx, qy, qz);
  Eigen::Isometry3d affine(quat.toRotationMatrix());
  affine.translation() = Eigen::Vector3d(x, y, z);
  transforms_[frame] = affine;
}

// ===========================
// DEPRECATED
// ===========================
bool cola2::ros::TransformHandler::getTransform(const std::string& frame, Eigen::Affine3d& transform)
{
  // call with isometry
  Eigen::Isometry3d trans;
  const bool success = getTransform(frame, trans);
  // transform to affine
  // transform = trans.affine();
  transform = Eigen::Affine3d(trans.rotation());
  transform.translation() = trans.translation();
  return success;
}

bool cola2::ros::TransformHandler::getDynamicTransform(const std::string& frame, Eigen::Affine3d& transform,
                                                       const ::ros::Time& time) const
{
  // call with isometry
  Eigen::Isometry3d trans;
  const bool success = getDynamicTransform(frame, trans, time);
  // transform to affine
  // transform = trans.affine();
  transform = Eigen::Affine3d(trans.rotation());
  transform.translation() = trans.translation();
  return success;
}
