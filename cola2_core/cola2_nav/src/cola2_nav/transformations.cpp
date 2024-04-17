/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/transformations.h"

#include <cola2_lib/utils/angles.h>

namespace transforms
{
// Position
Eigen::Vector3d position(const Eigen::Vector3d& measured_position, const Eigen::Quaterniond& body_orientation,
                         const Eigen::Vector3d& translation_from_origin)
{
  // z = zk - Rot(ori) * translation
  return measured_position - (body_orientation.toRotationMatrix() * translation_from_origin);
}

Eigen::Matrix3d positionCovariance(const Eigen::Matrix3d& measured_position_covariance,
                                   const Eigen::Matrix3d& origin_orientation_covariance,
                                   const Eigen::Quaterniond& origin_orientation,
                                   const Eigen::Vector3d& translation_from_origin)
{
  // z = zk - Rot(rpy) * translation
  // R = F1 Rk F1T + F2 Pori F2T
  // where:
  //   F1 = I
  //   F2 = df / drpy * translation
  const Eigen::Vector3d rpy = cola2::utils::quaternion2euler(origin_orientation);
  // construct F2
  Eigen::Matrix3d F2;
  F2.col(0) = -cola2::utils::d_rotation_d_roll(rpy) * translation_from_origin;
  F2.col(1) = -cola2::utils::d_rotation_d_pitch(rpy) * translation_from_origin;
  F2.col(2) = -cola2::utils::d_rotation_d_yaw(rpy) * translation_from_origin;
  // transform
  return measured_position_covariance + F2 * origin_orientation_covariance * F2.transpose();
}

// Orientation
Eigen::Quaterniond orientation(const Eigen::Quaterniond& measured_orientation,
                               const Eigen::Quaterniond& rotation_from_origin)
{
  // z = rotation * zk
  // measured_orientation is world2imu
  // rotation_from_origin in robot2imu
  return Eigen::Quaterniond(measured_orientation.toRotationMatrix() *
                            rotation_from_origin.toRotationMatrix().transpose());
}

Eigen::Matrix3d orientationCovariance(const Eigen::Matrix3d& measured_orientation_covariance,
                                      const Eigen::Quaterniond& rotation_from_origin)
{
  // z = rotation * zk
  // R = F1 Rk F1T
  // where:
  //   F1 = rotation
  const Eigen::Matrix3d F1 = rotation_from_origin.toRotationMatrix();
  return F1 * measured_orientation_covariance * F1.transpose();
}

// Linear velocity
Eigen::Vector3d linearVelocity(const Eigen::Vector3d& measured_linear_velocity,
                               const Eigen::Vector3d& origin_angular_velocity,
                               const Eigen::Quaterniond& rotation_from_origin,
                               const Eigen::Vector3d& translation_from_origin)
{
  // z = rotation * zk - wang x translation
  return rotation_from_origin * measured_linear_velocity - origin_angular_velocity.cross(translation_from_origin);
}

Eigen::Matrix3d linearVelocityCovariance(const Eigen::Matrix3d& measured_linear_velocity_covariance,
                                         const Eigen::Matrix3d& origin_angular_velocity_covariance,
                                         const Eigen::Quaterniond& rotation_from_origin,
                                         const Eigen::Vector3d& translation_from_origin)
{
  // z = rotation * zk - wang x translation
  // R = F1 Rk F1T + F2 Pwang F2T
  // where:
  //   F1 = rotation
  //   F2 = -[translation]T   (cross product matrix)
  const Eigen::Matrix3d F1 = rotation_from_origin.toRotationMatrix();
  const Eigen::Matrix3d F2 = -cola2::utils::cross_product_matrix(translation_from_origin);
  return F1 * measured_linear_velocity_covariance * F1.transpose() +
         F2 * origin_angular_velocity_covariance * F2.transpose();
}

// Angular velocity
Eigen::Vector3d angularVelocity(const Eigen::Vector3d& measured_angular_velocity,
                                const Eigen::Quaterniond& rotation_from_origin)
{
  // z = rotation * zk
  return rotation_from_origin.toRotationMatrix() * measured_angular_velocity;
}

Eigen::Matrix3d angularVelocityCovariance(const Eigen::Matrix3d& measured_angular_velocity_covariance,
                                          const Eigen::Quaterniond& rotation_from_origin)
{
  // z = rotation * zk
  // R = F1 Rk F1T
  // where:
  //   F1 = rotation
  const Eigen::Matrix3d F1 = rotation_from_origin.toRotationMatrix();
  return F1 * measured_angular_velocity_covariance * F1.transpose();
}

}  // namespace transforms
