/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_SURFACE_2D_H
#define COLA2_NAV_EKF_SURFACE_2D_H

#include <Eigen/Dense>
#include <string>
#include "./ekf_base_ros.h"

/**
 * \brief Final definitions on how the filter state is defined.
 *
 * In this case the state contains position [x y] orientation [yaw] velocities [u v] and angular rate [vyaw]. Other
 * orientations and anglular velocities are saved directly from the sensor.
 * State vector:
 *  x = [x y yaw vx vy vyaw]
 * with velocities in body frame.
 */
class EKFSurface2D : public EKFBaseROS
{
private:
  // Auxiliary states not in the state vector x
  Eigen::Vector3d rpy_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_vel_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rpy_cov_ = 0.1 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d ang_vel_cov_ = 0.1 * Eigen::Matrix3d::Identity();

protected:
  // *****************************************
  // Implemented methods from EKFBase
  // *****************************************
  void setPositionXY(const Eigen::Vector2d& xy) final;
  void normalizeState() final;
  void computePredictionMatrices(const double dt) final;
  bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) final;
  bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) final;
  bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) final;
  bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                      const bool from_dvl = true) final;
  bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) final;

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   *  \brief Constructor.
   */
  EKFSurface2D();
  /**
   *  \brief Destructor.
   */
  ~EKFSurface2D() = default;

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

#endif  // COLA2_NAV_EKF_SURFACE_2D_H
