/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_position_velocity.h"

EKFPositionVelocity::EKFPositionVelocity(const bool online) : EKFBaseROS(6, online)
{
  // state vector contains [x y z vx vy vz] = size -> 6
  // noise process matrix Q_ => size 3x3 (noise in acceleration)
}

void EKFPositionVelocity::setPositionXY(const Eigen::Vector2d& xy)
{
  x_.head(2) = xy;
}

void EKFPositionVelocity::normalizeState()
{
  // no angles in state
}

void EKFPositionVelocity::computePredictionMatrices(const double dt)
{
  const unsigned int size = x_.rows();
  const double dt2 = (dt * dt) / 2.0;
  const Eigen::Matrix3d rot = getRotation();

  // A is the jacobian matrix of f(x)
  //   [ I_3x3   Rot(rpy)*dt]
  //   [ 0_3x3    I_3x3     ]
  A_ = Eigen::MatrixXd::Identity(size, size);
  A_.block<3, 3>(0, 3) = dt * rot;

  // The noise in the system is a term added to the acceleration:
  // e.g. x[0] = x1 + std::cos(pitch)*std::cos(yaw)*(vx1*t +  Eax*t^2/2)..
  // then, dEax/dt of x[0] = std::cos(pitch)*std::cos(yaw)*t^2/2
  //
  // W_6x3 = [dt2/2 * Rot(rpy)]
  //         [   dt * I_3x3   ]
  assert((Q_.rows() == 3) && (Q_.cols() == 3));  // check expected size
  W_ = Eigen::MatrixXd::Zero(size, Q_.rows());
  W_.block<3, 3>(0, 0) = dt2 * rot;
  W_.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();

  // Compute Prediction Model with constant velocity
  // The model takes as state 3D position (x, y, z) and linear velocity (vx, vy, vz).
  // The input is the orientation (roll, pitch yaw) and the linear accelerations (ax, ay, az).
  // f(x) = [x + Rot(rpy) * v * dt]
  //        [       v_3x3         ]
  const Eigen::Vector3d pos = getPosition();
  const Eigen::Vector3d vel = getVelocity();
  fx_ = x_;
  fx_.head(3) = pos + rot * vel * dt;
}

bool EKFPositionVelocity::updatePositionXY(const double, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov)
{
  // Initialization
  if (!init_ekf_)
  {
    x_.head(2) = pose_xy;
    init_gps_ = true;  // TODO: also true when getting a USBL
    ROS_INFO_ONCE("gps init");
    return true;
  }
  // Update
  unsigned int size = x_.rows();
  Eigen::Vector2d h = x_.head(2);                      // h(x) = xy
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, size);  // H = dh(x) / dx
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  return applyUpdate(pose_xy - h, H, cov, Eigen::Matrix2d::Identity(), 30.0);
}
bool EKFPositionVelocity::updatePositionZ(const double, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov)
{
  // Initialization
  if (!init_ekf_)
  {
    x_(2) = pose_z(0);
    init_depth_ = true;
    ROS_INFO_ONCE("depth init");
    return true;
  }
  // Update
  unsigned int size = x_.rows();
  Eigen::Vector1d h = x_.segment<1>(2);                // h(x) = z
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, size);  // H = dh(x) / dx
  H(0, 2) = 1;
  return applyUpdate(pose_z - h, H, cov, Eigen::Matrix1d::Identity(), 30.0);
}
bool EKFPositionVelocity::updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov)
{
  // Initialization
  if (!init_ekf_)
  {
    if ((!config_.use_gps_data_ || init_gps_) && (init_depth_) && (init_dvl_))
    {
      last_ekf_init_time_ = t;
      init_ekf_ = true;
      ROS_INFO_ONCE("ekf init");
    }
    rpy_ = rpy;
    rpy_cov_ = cov;
    init_imu_ = true;
    ROS_INFO_ONCE("imu init");
    return true;
  }
  // Update (used as true reading)
  rpy_ = rpy;
  rpy_cov_ = cov;
  return true;
}
bool EKFPositionVelocity::updateVelocity(const double, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                                         const bool from_dvl)
{
  // Initialization
  if (!init_ekf_)
  {
    if (from_dvl)
    {
      x_.segment<3>(3) = vel;
      init_dvl_ = true;
      ROS_INFO_ONCE("dvl init");
    }
    return true;
  }
  // Update
  unsigned int size = x_.rows();
  Eigen::Vector3d h = x_.segment<3>(3);                // h(x) = vx vy vz
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, size);  // H = dh(x) / dx
  H(0, 3) = 1;
  H(1, 4) = 1;
  H(2, 5) = 1;
  return applyUpdate(vel - h, H, cov, Eigen::Matrix3d::Identity(), 16.0);
}
bool EKFPositionVelocity::updateOrientationRate(const double, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov)
{
  // Update (used as true reading)
  ang_vel_ = rate;
  ang_vel_cov_ = cov;
  return true;
}

Eigen::Vector3d EKFPositionVelocity::getPosition() const
{
  return x_.head(3);
}
Eigen::Vector3d EKFPositionVelocity::getVelocity() const
{
  return x_.segment<3>(3);
}
Eigen::Vector3d EKFPositionVelocity::getEuler() const
{
  return rpy_;
}
Eigen::Vector3d EKFPositionVelocity::getAngularVelocity() const
{
  return ang_vel_;
}
Eigen::Matrix3d EKFPositionVelocity::getPositionUncertainty() const
{
  return P_.topLeftCorner(3, 3);
}
Eigen::Matrix3d EKFPositionVelocity::getVelocityUncertainty() const
{
  return P_.block<3, 3>(3, 3);
}
Eigen::Matrix3d EKFPositionVelocity::getOrientationUncertainty() const
{
  return rpy_cov_;
}
Eigen::Matrix3d EKFPositionVelocity::getAngularVelocityUncertainty() const
{
  return ang_vel_cov_;
}
