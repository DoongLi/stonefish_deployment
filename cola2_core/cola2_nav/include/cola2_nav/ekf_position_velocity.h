/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_POSITION_VELOCITY_H
#define COLA2_NAV_EKF_POSITION_VELOCITY_H

#include <Eigen/Dense>
#include <string>
#include "./ekf_base_ros.h"

/**
 * \brief Final definitions on how the filter state is defined.
 *
 * In this case the state contains position [x y z] and velocities [u v w]. Orientation and angular velocity are saved
 * directly from the sensor.
 *
 *   - state vector contains [x y z vx vy vz] = size -> 6
 *   - noise process matrix Q_ => size 3x3 (noise in acceleration)
 */
class EKFPositionVelocity : public EKFBaseROS
{
private:
  // Auxiliary states not in the state vector x
  Eigen::Vector3d rpy_ = Eigen::Vector3d::Zero();                    //!< Current filter orientation
  Eigen::Vector3d ang_vel_ = Eigen::Vector3d::Zero();                //!< Current filter angular velocity
  Eigen::Matrix3d rpy_cov_ = 0.1 * Eigen::Matrix3d::Identity();      //!< Current filter orientation covariance
  Eigen::Matrix3d ang_vel_cov_ = 0.1 * Eigen::Matrix3d::Identity();  //!< Current filter angular velocity covariance

protected:
  // *****************************************
  // Implemented methods from EKFBase
  // *****************************************
  // cannot change
  void setPositionXY(const Eigen::Vector2d& xy) final;
  bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) final;
  bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) final;
  bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) final;
  bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                      const bool from_dvl = true) final;
  bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) final;
  // can change if we add features
  void normalizeState() override;
  void computePredictionMatrices(const double dt) override;

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   * \brief Constructor.
   *
   * \param online Run the filter online (enable publishers, subscribers and services that require init_node)
   */
  explicit EKFPositionVelocity(const bool online = true);
  /**
   * \brief Destructor.
   */
  ~EKFPositionVelocity() = default;

  // *****************************************
  // Implemented methods from EKFBase
  // *****************************************
  Eigen::Vector3d getPosition() const final;
  Eigen::Vector3d getVelocity() const final;
  Eigen::Vector3d getEuler() const final;
  Eigen::Vector3d getAngularVelocity() const final;
  Eigen::Matrix3d getPositionUncertainty() const final;
  Eigen::Matrix3d getVelocityUncertainty() const final;
  Eigen::Matrix3d getOrientationUncertainty() const final;
  Eigen::Matrix3d getAngularVelocityUncertainty() const final;
};

#endif  // COLA2_NAV_EKF_POSITION_VELOCITY_H
