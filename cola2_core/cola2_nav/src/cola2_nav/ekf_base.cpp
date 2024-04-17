/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_base.h"

EKFBase::EKFBase(const unsigned int initial_state_vector_size) : initial_state_vector_size_(initial_state_vector_size)
{
  x_ = Eigen::VectorXd::Zero(initial_state_vector_size_);
  P_ = Eigen::MatrixXd::Identity(initial_state_vector_size_, initial_state_vector_size_);
}

bool EKFBase::makePrediction(const double now)
{
  if (init_ekf_)
  {
    const double dt = now - last_prediction_;  // time increment
    if (dt > 0.0 && dt <= 1.0)
    {
      computePredictionMatrices(dt);
      // x = f(x, u)
      // P = A P AT + W Q WT
      x_ = fx_;
      normalizeState();
      P_ = A_ * P_ * A_.transpose() + W_ * Q_ * W_.transpose();
      last_prediction_ = now;
      return true;
    }
    else if (dt > -0.15 && dt <= 0.0)
    {
      // state is the same
      return true;
    }
    else
    {
      std::cerr << "makePrediction invalid period " << dt << "\n";
      last_prediction_ = now;  // Update time, otherwise everything will fail!
      return false;
    }
  }
  else
  {
    last_prediction_ = now;  // Update time, otherwise everything will fail!
    return false;
  }
}

bool EKFBase::applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                          const Eigen::MatrixXd& V, const double mahalanobis_distance_threshold)
{
  // Compute distance
  const double distance = mahalanobisDistance(innovation, R, H, V);
  // No threshold or distance below threshold
  const bool mahalanobis_ok = (mahalanobis_distance_threshold < 0.0) || (distance < mahalanobis_distance_threshold);
  if (filter_updates_ < 100 || mahalanobis_ok)
  {
    // Compute updated state vector
    const Eigen::MatrixXd S = H * P_ * H.transpose() + V * R * V.transpose();
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ += K * innovation;
    normalizeState();
    const int size = x_.rows();
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(size, size) - K * H;
    // P_ = (Eigen::MatrixXd::Identity(size, size) - K * H) * P_;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();  // Joseph form
    ++filter_updates_;
    // Check integrity
    checkIntegrity();
    // Update applied
    return true;
  }
  return false;
}

void EKFBase::showStateVector() const
{
  std::cout << "x: " << x_.transpose() << '\n';
  std::cout << "P:\n" << P_ << '\n';
}

Eigen::VectorXd EKFBase::getStateVector() const
{
  return x_;
}

Eigen::MatrixXd EKFBase::getCovarianceMatrix() const
{
  return P_;
}

Eigen::Isometry3d EKFBase::getTransform() const
{
  Eigen::Isometry3d trans(getRotation());
  trans.translation() = getPosition();
  return trans;
}

Eigen::Matrix3d EKFBase::getRotation() const
{
  return getOrientation().toRotationMatrix();
}

Eigen::Quaterniond EKFBase::getOrientation() const
{
  return cola2::utils::euler2quaternion(getEuler());
}

bool EKFBase::isInitialized() const
{
  return init_ekf_;
}

double EKFBase::mahalanobisDistance(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& R,
                                    const Eigen::MatrixXd& H, const Eigen::MatrixXd& V)
{
  const Eigen::MatrixXd S = H * P_ * H.transpose() + V * R * V.transpose();
  const Eigen::VectorXd d = innovation.transpose() * S.inverse() * innovation;
  return std::sqrt(d(0, 0));
}

void EKFBase::checkIntegrity()
{
  // NaN check
  for (unsigned int i = 0; i < x_.rows(); i++)
  {
    if (std::isnan(x_(i)))
    {
      std::cout << "\033[1;31m"
                << "NaNs detected!!!"
                << "\033[0m\n";
    }
    if (P_(i, i) < 0.0)
    {
      P_(i, i) = 0.01;
      std::cout << "Negative values in P(" << i << "," << i << ")\n";
    }
  }
}
