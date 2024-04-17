/**
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_TRANSFORMATIONS_H
#define COLA2_NAV_TRANSFORMATIONS_H

#include <cola2_lib/utils/angles.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace transforms
{
/**
 * \brief Transform a position measured in one frame to another frame
 *
 * \param measured_position Measured position in the sensor frame
 * \param body_orientation Orientation of the rigid body that connects both frames
 * \param translation_from_origin Translation from the new frame to the mesauring frame in new frame coordinates
 * \return Position in the other frame
 */
Eigen::Vector3d position(const Eigen::Vector3d& measured_position, const Eigen::Quaterniond& body_orientation,
                         const Eigen::Vector3d& translation_from_origin);

/**
 * \brief Transform a position covariance measured in one frame to another frame
 *
 * \param measured_position_covariance Measured position in the sensor frame
 * \param origin_orientation_covariance Orientation covariance of the rigid body that connects both frames
 * \param origin_orientation Orientation from the new frame to the mesauring frame in new frame coordinates
 * \param translation_from_origin Translation from the new frame to the mesauring frame in new frame coordinates
 * \return Covariance in the other frame
 */
Eigen::Matrix3d positionCovariance(const Eigen::Matrix3d& measured_position_covariance,
                                   const Eigen::Matrix3d& origin_orientation_covariance,
                                   const Eigen::Quaterniond& origin_orientation,
                                   const Eigen::Vector3d& translation_from_origin);

/**
 * \brief Transform an orientation measured in one frame to another frame
 *
 * \param measured_orientation Measured orientation in the sensor frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \return Orientation in the other frame
 */
Eigen::Quaterniond orientation(const Eigen::Quaterniond& measured_orientation,
                               const Eigen::Quaterniond& rotation_from_origin);

/**
 * \brief Transform an orientation covariance measured in one frame to another frame
 *
 * \param measured_orientation_covariance Measured orientation covariance in the sensor frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \return Covariance in the other frame
 */
Eigen::Matrix3d orientationCovariance(const Eigen::Matrix3d& measured_orientation_covariance,
                                      const Eigen::Quaterniond& rotation_from_origin);

/**
 * \brief Transform a linear velocity measured in one frame to another frame
 *
 * \param measured_linear_velocity Measured linear velocity in the sensor frame
 * \param origin_angular_velocity Angular velocity in the new frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \param translation_from_origin Translation from the new frame to the mesauring frame in new frame coordinates
 * \return Linear velocity in the other frame
 */
Eigen::Vector3d linearVelocity(const Eigen::Vector3d& measured_linear_velocity,
                               const Eigen::Vector3d& origin_angular_velocity,
                               const Eigen::Quaterniond& rotation_from_origin,
                               const Eigen::Vector3d& translation_from_origin);

/**
 * \brief Transform a linear velocity covariance measured in one frame to another frame
 *
 * \param measured_linear_velocity_covariance Measured linear velocity in the sensor frame
 * \param origin_angular_velocity_covariance Angular velocity covariance in the new frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \param translation_from_origin Translation from the new frame to the mesauring frame in new frame coordinates
 * \return Covariance in the other frame
 */
Eigen::Matrix3d linearVelocityCovariance(const Eigen::Matrix3d& measured_linear_velocity_covariance,
                                         const Eigen::Matrix3d& origin_angular_velocity_covariance,
                                         const Eigen::Quaterniond& rotation_from_origin,
                                         const Eigen::Vector3d& translation_from_origin);

/**
 * \brief Transform an angular velocity measured in one frame to another frame
 *
 * \param measured_angular_velocity Measured angular velocity in the sensor frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \return Angular velocity in the other frame
 */
Eigen::Vector3d angularVelocity(const Eigen::Vector3d& measured_angular_velocity,
                                const Eigen::Quaterniond& rotation_from_origin);

/**
 * \brief Transform an angular velocity covariance measured in one frame to another frame
 *
 * \param measured_angular_velocity_covariance Measured angular velocity covariance in the sensor frame
 * \param rotation_from_origin Rotation from the new frame to the mesauring frame in new frame coordinates
 * \return Covariance in the other frame
 */
Eigen::Matrix3d angularVelocityCovariance(const Eigen::Matrix3d& measured_angular_velocity_covariance,
                                          const Eigen::Quaterniond& rotation_from_origin);

}  // namespace transforms

#endif  // COLA2_NAV_TRANSFORMATIONS_H
