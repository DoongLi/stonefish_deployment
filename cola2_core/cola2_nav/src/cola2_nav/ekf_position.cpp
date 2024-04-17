/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_position.h"

EKFPosition::EKFPosition(const bool online) : EKFBaseROS(3, online)
{
  // state vector contains [x y z] = size -> 3
  // noise process matrix Q_ => size 3x3 (noise in velocity)
}

void EKFPosition::setPositionXY(const Eigen::Vector2d& xy)
{
  x_.head(2) = xy;
}

void EKFPosition::normalizeState()
{
  // no angles in state
}

void EKFPosition::computePredictionMatrices(const double dt)
{
  const unsigned int size = x_.rows();
  const Eigen::Matrix3d rot = getRotation();

  // A is the jacobian matrix of f(x)
  //   [ I_3x3 ]
  A_ = Eigen::MatrixXd::Identity(size, size);

  // The noise in the system is a term added to the velocity:
  // e.g. x[0] = x1 + std::cos(pitch)*std::cos(yaw)*(vx1*t +  Eax*t^2/2)..
  // then, dEax/dt of x[0] = std::cos(pitch)*std::cos(yaw)*t^2/2
  //
  Q_ = getVelocityUncertainty();
  assert((Q_.rows() == 3) && (Q_.cols() == 3));  // check expected size

  // W_3x3 = [dt * Rot(rpy)]
  W_ = Eigen::MatrixXd::Zero(size, Q_.rows());
  W_.block<3, 3>(0, 0) = dt * rot;

  // Compute Prediction Model with constant velocity
  // The model takes as state 3D position (x, y, z) and linear velocity (vx, vy, vz).
  // The input is the orientation (roll, pitch yaw) and the linear accelerations (ax, ay, az).
  // f(x) = [x + Rot(rpy) * (v + nv) * dt]
  const Eigen::Vector3d pos = getPosition();
  const Eigen::Vector3d vel = getVelocity();
  //   fx_ = x_;
  fx_ = pos + rot * vel * dt;
}

bool EKFPosition::updatePositionXY(const double, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov)
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
bool EKFPosition::updatePositionZ(const double, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov)
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
bool EKFPosition::updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov)
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
bool EKFPosition::updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                                 const bool from_dvl)
{
  static const Eigen::Vector3d MAX_ACCEL{ 2.0, 1.0, 1.0 };
  static double last_t = 0.0;
  static Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();
  // Initialization
  if (!init_ekf_)
  {
    vel_ = vel;
    cov_vel_ = cov;
    if (from_dvl)
    {
      last_t = t;
      last_vel = vel;
      init_dvl_ = true;
      ROS_INFO_ONCE("dvl init");
    }
    return true;
  }
  // Update fallback
  if (!from_dvl)
  {
    vel_ = vel;
    cov_vel_ = cov;
    return true;
  }
  // Update dvl
  const double dt = t - last_t;
  const Eigen::Vector3d acceleration = (vel - last_vel).array().abs() / dt;
  if ((acceleration.array() < MAX_ACCEL.array()).all())
  {
    last_t = t;
    last_vel = vel;
    vel_ = vel;
    cov_vel_ = cov;
    return true;
  }
  ROS_WARN("Too big acceleration (%.2f, %.2f, %.2f) in time %.3f with new velocity (%.2f, %.2f, %.2f)", acceleration(0),
           acceleration(1), acceleration(2), dt, vel(0), vel(1), vel(2));
  return false;
}

bool EKFPosition::updateOrientationRate(const double, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov)
{
  // Update (used as true reading)
  ang_vel_ = rate;
  ang_vel_cov_ = cov;
  return true;
}

Eigen::Vector3d EKFPosition::getPosition() const
{
  return x_.head(3);
}
Eigen::Vector3d EKFPosition::getVelocity() const
{
  return vel_;
}
Eigen::Vector3d EKFPosition::getEuler() const
{
  return rpy_;
}
Eigen::Vector3d EKFPosition::getAngularVelocity() const
{
  return ang_vel_;
}
Eigen::Matrix3d EKFPosition::getPositionUncertainty() const
{
  return P_.topLeftCorner(3, 3);
}
Eigen::Matrix3d EKFPosition::getVelocityUncertainty() const
{
  return cov_vel_;
}
Eigen::Matrix3d EKFPosition::getOrientationUncertainty() const
{
  return rpy_cov_;
}
Eigen::Matrix3d EKFPosition::getAngularVelocityUncertainty() const
{
  return ang_vel_cov_;
}
