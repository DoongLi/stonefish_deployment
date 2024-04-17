/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_surface_2d.h"

EKFSurface2D::EKFSurface2D() : EKFBaseROS(6)
{
  // state vector contains [x y yaw vx vy vyaw] = size -> 6
  P_(0, 0) = 0.1;  // cov x
  P_(1, 1) = 0.1;  // cov y
  P_(2, 2) = 0.1;  // cov yaw
  P_(3, 3) = 0.0;  // cov vx (vehicle frame)
  P_(4, 4) = 0.0;  // cov vy (vehicle frame)
  P_(5, 5) = 0.0;  // cov vyaw
}

void EKFSurface2D::setPositionXY(const Eigen::Vector2d& xy)
{
  x_.head(2) = xy;
}

void EKFSurface2D::normalizeState()
{
  // normalize yaw
  x_(2) = cola2::utils::wrapAngle(x_(2));
}

void EKFSurface2D::computePredictionMatrices(const double dt)
{
  const Eigen::Vector3d rpy = getEuler();
  const double sy = std::sin(rpy(2));  // std::sin(yaw)
  const double cy = std::cos(rpy(2));  // std::cos(yaw)
  const unsigned int size = x_.rows();
  const double dt2 = (dt * dt) / 2.0;
  const Eigen::Vector3d vel = getVelocity();
  const double vx = vel(0);
  const double vy = vel(1);

  // State prediction
  // x = x + (vx * dt + nvx * dt2) * cy - (vy * dt + nvy * dt2) * sy
  // y = y + (vx * dt + nvx * dt2) * sy + (vy * dt + nvy * dt2) * cy
  // yaw = yaw + vyaw * dt + nvyaw * dt2
  // vx = vx + nvx * dt
  // vy = vy + nvy * dt
  // vyaw = vyaw + nvyaw * dt

  // A is the jacobian matrix of f(x)
  A_ = Eigen::MatrixXd::Identity(size, size);
  A_(0, 2) = -vx * dt * sy - vy * dt * cy;  // fx/yaw
  A_(0, 3) = +dt * cy;                      // fx/vx
  A_(0, 4) = -dt * sy;                      // fx/vy

  A_(1, 2) = +vx * dt * cy - vy * dt * sy;  // fy/yaw
  A_(1, 3) = +dt * sy;                      // fy/vx
  A_(1, 4) = +dt * cy;                      // fy/vy

  A_(2, 5) = dt;  // fyaw/vyaw

  // The noise in the system is in vx, vy and vyaw => W(6,3)
  W_ = Eigen::MatrixXd::Zero(size, Q_.rows());
  W_(0, 0) = +dt2 * cy;  // fx/nvx
  W_(0, 1) = -dt2 * sy;  // fx/nvy

  W_(1, 0) = +dt2 * sy;  // fy/nvx
  W_(1, 1) = +dt2 * cy;  // fy/nvy

  W_(2, 2) = dt2;  // fyaw/nvyaw

  W_(3, 0) = dt;  // fvx/nvx
  W_(4, 1) = dt;  // fvy/nvy
  W_(5, 2) = dt;  // fvyaw/nvyaw

  // Prediction Model with constant velocity
  fx_ = x_;
  fx_(0) += vx * dt * cy - vy * dt * sy;
  fx_(1) += vx * dt * sy + vy * dt * cy;
  fx_(2) += x_(5) * dt;
  fx_(2) = cola2::utils::wrapAngle(fx_(2));
}

bool EKFSurface2D::updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov)
{
  // Check usage
  if (!config_.use_gps_data_)
  {
    return false;
  }
  // Update time
  last_gps_time_ = t;
  // Initialization
  if (!init_ekf_)
  {
    x_.head(2) = pose_xy;
    init_gps_ = true;
    ROS_INFO_ONCE("gps init");
    return true;
  }
  // Update
  const unsigned int size = x_.rows();
  const Eigen::Vector2d h = x_.head(2);                // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, size);  // H = dh(x)/dx
  H(0, 0) = 1;
  H(1, 1) = 1;
  return applyUpdate(pose_xy - h, H, cov, Eigen::Matrix2d::Identity(), 30.0);
}
bool EKFSurface2D::updatePositionZ(const double, const Eigen::Vector1d&, const Eigen::Matrix1d&)
{
  // Check usage
  if (!config_.use_depth_data_)
  {
    return false;
  }
  ROS_FATAL("updatePositionZ() not implemented");
  return false;
}
bool EKFSurface2D::updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov)
{
  // Update time
  last_imu_time_ = t;
  // Initialization
  if (!init_ekf_)
  {
    if (((!config_.use_gps_data_) || (init_gps_)) && ((!config_.use_dvl_data_) || (init_dvl_)))
    {
      init_ekf_ = true;
      ROS_INFO_ONCE("ekf init");
    }
    x_(2) = rpy(2);
    rpy_ = rpy;
    rpy_cov_ = cov;
    init_imu_ = true;
    ROS_INFO_ONCE("imu init");
    return true;
  }
  // Update (used as true reading)
  rpy_ = rpy;
  rpy_cov_ = cov;
  // Update
  const unsigned int size = x_.rows();
  const Eigen::Vector1d h = x_.segment<1>(2);          // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, size);  // H = dh(x)/dx
  H(0, 2) = 1;
  Eigen::Vector1d inno = rpy.tail(1) - h;
  inno(0) = cola2::utils::wrapAngle(inno(0));
  return applyUpdate(inno, H, cov.bottomRightCorner(1, 1), Eigen::Matrix1d::Identity(), 30.0);
}
bool EKFSurface2D::updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                                  const bool from_dvl)
{
  // Check usage
  if (!config_.use_dvl_data_)
  {
    return false;
  }
  // Update time
  if (from_dvl)
  {
    last_dvl_time_ = t;
  }
  // Initialization
  if (!init_ekf_)
  {
    if (from_dvl)
    {
      x_.segment<2>(3) = vel.head(2);
      init_dvl_ = true;
      ROS_INFO_ONCE("dvl init");
    }
    return true;
  }
  // Update
  const unsigned int size = x_.rows();
  const Eigen::Vector2d h = x_.segment<2>(3);          // h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, size);  // H = dh(x) / dx
  H(0, 3) = 1;
  H(1, 4) = 1;
  return applyUpdate(vel.head(2) - h, H, cov.topLeftCorner(2, 2), Eigen::Matrix2d::Identity(), 16.0);
}
bool EKFSurface2D::updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov)
{
  // Update time
  last_imu_time_ = t;
  // Update (used as true reading)
  ang_vel_ = rate;
  ang_vel_cov_ = cov;
  // Üpdate
  const unsigned int size = x_.rows();
  const Eigen::Vector1d h = x_.tail(1);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, size);
  H(0, 5) = 1;
  return applyUpdate(rate.tail(1) - h, H, cov.bottomRightCorner(1, 1), Eigen::Matrix1d::Identity(), 16.0);
}

Eigen::Vector3d EKFSurface2D::getPosition() const
{
  return Eigen::Vector3d(x_(0), x_(1), 0.0);
}
Eigen::Vector3d EKFSurface2D::getVelocity() const
{
  return Eigen::Vector3d(x_(3), x_(4), 0.0);
}
Eigen::Vector3d EKFSurface2D::getEuler() const
{
  return Eigen::Vector3d(rpy_(0), rpy_(1), x_(2));
}
Eigen::Vector3d EKFSurface2D::getAngularVelocity() const
{
  return Eigen::Vector3d(ang_vel_(0), ang_vel_(1), x_(5));
}
Eigen::Matrix3d EKFSurface2D::getPositionUncertainty() const
{
  Eigen::Matrix3d cov = 0.01 * Eigen::Matrix3d::Identity();
  cov.topLeftCorner(2, 2) = P_.topLeftCorner(2, 2);
  return cov;
}
Eigen::Matrix3d EKFSurface2D::getVelocityUncertainty() const
{
  Eigen::Matrix3d cov = 0.01 * Eigen::Matrix3d::Identity();
  cov.topLeftCorner(2, 2) = P_.block<2, 2>(3, 3);
  return cov;
}
Eigen::Matrix3d EKFSurface2D::getOrientationUncertainty() const
{
  Eigen::Matrix3d cov = rpy_cov_;
  cov(2, 2) = P_(2, 2);
  return cov;
}
Eigen::Matrix3d EKFSurface2D::getAngularVelocityUncertainty() const
{
  Eigen::Matrix3d cov = ang_vel_cov_;
  cov(2, 2) = P_(5, 5);
  return cov;
}
