
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_BASE_H
#define COLA2_NAV_EKF_BASE_H

#include <cola2_lib/utils/angles.h>
#include <Eigen/Dense>
#include <iostream>

namespace
{
constexpr double MAX_COVARIANCE = 0.025;
}

namespace Eigen
{
using Vector1d = Matrix<double, 1, 1>;  //!< Eigen vector of size 1x1
using Matrix1d = Matrix<double, 1, 1>;  //!< Eigen matrix of size 1x1
using Vector6d = Matrix<double, 6, 1>;  //!< Eigen vector of size 6x1
using Matrix6d = Matrix<double, 6, 6>;  //!< Eigen matrix of size 6x6
}  // namespace Eigen

/**
 * \brief EKF base class with all basic equations.
 */
class EKFBase
{
private:
  /**
   * \brief Compute Mahalanobis distance for the innovation.
   *
   * \param innovation Innovation of the measurement (innovation = z - h(x))
   * \param R Covariance of the measurement
   * \param H Linearization of h(x) w.r.t. filter state (dh(x) / dx)
   * \param V Linearization of h(x) w.r.t. measurement noise (dh(x) / dnoise)
   * \return Mahalanobis distance
   */
  double mahalanobisDistance(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H,
                             const Eigen::MatrixXd& V);
  /**
   * \brief Check integrity of the state vector and the diagonal of the covariance matrix.
   */
  void checkIntegrity();

protected:
  const unsigned int initial_state_vector_size_;  //!< size of state vector
  Eigen::VectorXd x_;                             //!< state vector
  Eigen::MatrixXd P_;                             //!< covariance matrix
  Eigen::MatrixXd Q_;                             //!< noise prediction matrix
  bool init_ekf_ = false;                         //!< filter initialized
  size_t filter_updates_ = 0;                     //!< number of updates already done
  double last_prediction_ = 0.0;                  //!< time of last prediction

  Eigen::VectorXd fx_;  //!< State prediction function f(x) (computed in computePredictionMatrices)
  Eigen::MatrixXd A_;   //!< Linearizartion of f(x) w.r.t. filter state (df(x) / dx) (computed in
                        //!< computePredictionMatrices)
  Eigen::MatrixXd W_;   //!< Linearizartion of f(x) w.r.t. process noise (df(x) / dnoise) (computed in
                        //!< computePredictionMatrices)

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   * \brief EKF base where the basic equations are applied.
   *
   *  \param initial_state_vector_size Size in rows of the state vector
   */
  explicit EKFBase(const unsigned int initial_state_vector_size);

  /**
   * \brief EKF base virtual destructor.
   */
  virtual ~EKFBase() = default;

  // *****************************************
  // Basic EKF
  // *****************************************
  /**
   * \brief Predict the filter until the specified time.
   *
   *  The prediction equations are:
   *    x = f(x)
   *    P = A P AT + W Q WT
   *    where:
   *      A = df / dx = constant velocity model
   *      W = df / dnoise where noise is linear acceleration
   *
   *  \param now Time where we want to predict the filter
   *  \return Success on making the prediction
   */
  bool makePrediction(const double now);

  /**
   * \brief Update the filter according to a measurement.
   *
   *  The update equations are:
   *    innovation = z - h(x)
   *    S = H P HT + V R VT
   *    K = P HT S-1
   *    x = K innovation
   *    P = (I - K H) P (I - K H)T + K R KT
   *    where:
   *      H = dh / dx
   *      V = dh / dnoise where noise is the noise of the measurement (usually V = I)
   *
   *  \param innovation Innovation of the measurement (innovation = z - h(x))
   * \param H Linearization of h(x) w.r.t. filter state (dh(x) / dx)
   * \param R Covariance of the measurement
   * \param V Linearization of h(x) w.r.t. measurement noise (dh(x) / dnoise)
   * \param mahalanobis_distance_threshold Upper threshold to apply the update (no threshold if value < 0.0)
   * \return Success on applying the update. Fails on not meeting the threshold.
   */
  bool applyUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                   const Eigen::MatrixXd& V, const double mahalanobis_distance_threshold);

  // *****************************************
  // Getters
  // *****************************************
  /**
   * \brief Print state vector x and covariance matrix onscreen.
   */
  void showStateVector() const;
  /**
   * \brief Get state vector x.
   *
   *  \return State vector.
   */
  Eigen::VectorXd getStateVector() const;
  /**
   * \brief Get covariance matrix P.
   *
   *  \return State covariance.
   */
  Eigen::MatrixXd getCovarianceMatrix() const;
  /**
   * \brief Get transform world to vehicle.
   *
   * \return Transform world2vehicle
   */
  Eigen::Isometry3d getTransform() const;
  /**
   * \brief Get rotation matrix according to filter orientation.
   *
   * \return Rotation matrix corresponding to filter orientation
   */
  Eigen::Matrix3d getRotation() const;
  /**
   * \brief Get orientation quaternion according to filter orientation.
   *
   * \return Orientation of the filter as a quaternion
   */
  Eigen::Quaterniond getOrientation() const;
  /**
   * \brief Return whether the ekf filter is initialized.
   *
   * \return Filter initialization
   */
  bool isInitialized() const;

  // *****************************************
  // To be implemented
  // *****************************************
protected:
  /**
   * \brief Set position xy.
   *
   * \param xy Position xy to write to filter
   */
  virtual void setPositionXY(const Eigen::Vector2d& xy) = 0;
  /**
   * \brief Compute prediction matrices fx_, A_, W_.
   *
   * \param dt Time increment since last prediction
   */
  virtual void computePredictionMatrices(const double dt) = 0;
  /**
   * \brief Normalize state vector x (when it has orientation).
   */
  virtual void normalizeState() = 0;
  /**
   * \brief Update state from a GPS measurement.
   *
   * \param t Timestamp of the measurement
   * \param pose_xy Measurement in NED coodinates in vehicle frame
   * \param cov Covariance of the measurement
   * \return Success of the update
   */
  virtual bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) = 0;
  /**
   * \brief Update state from a Depth measurement.
   *
   * \param t Timestamp of the measurement
   * \param pose_z Measurement in vehicle frame
   * \param cov Covariance of the measurement
   * \return Success of the update
   */
  virtual bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) = 0;
  /**
   * \brief Update state from a Orientation measurement.
   *
   * \param t Timestamp of the measurement
   * \param rpy Measurement in Euler angles in vehicle frame
   * \param cov Covariance of the measurement
   * \return Success of the update
   */
  virtual bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) = 0;
  /**
   * \brief Update state from a DVL measurement.
   *
   * \param t Timestamp of the measurement
   * \param vel Measurement in vehicle frame
   * \param cov Covariance of the measurement
   * \return Success of the update
   */
  virtual bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                              const bool from_dvl = true) = 0;
  /**
   * \brief Update state from a OrientationRate measurement.
   *
   * \param t Timestamp of the measurement
   * \param rate Measurement in vehicle frame
   * \param cov Covariance of the measurement
   * \return Success of the update
   */
  virtual bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) = 0;

public:
  /**
   * \brief Get position.
   *
   * \return Current position xyz of the filter
   */
  virtual Eigen::Vector3d getPosition() const = 0;
  /**
   * \brief Get velocity.
   *
   * \return Current linear velocity of the filter
   */
  virtual Eigen::Vector3d getVelocity() const = 0;
  /**
   * \brief Get euler orientation.
   *
   * \return Current orientation of the filter in Euler angles
   */
  virtual Eigen::Vector3d getEuler() const = 0;
  /**
   * \brief Get angular velocity.
   *
   * \return Current angular velocity of the filter
   */
  virtual Eigen::Vector3d getAngularVelocity() const = 0;
  /**
   * \brief Get position uncertainty.
   *
   * \return Current position covariance
   */
  virtual Eigen::Matrix3d getPositionUncertainty() const = 0;
  /**
   * \brief Get velocity uncertainty.
   *
   * \return Current velocity covariance
   */
  virtual Eigen::Matrix3d getVelocityUncertainty() const = 0;
  /**
   * \brief Get orientation uncertainty.
   *
   * \return Current orientation covariance
   */
  virtual Eigen::Matrix3d getOrientationUncertainty() const = 0;
  /**
   * \brief Get angular velocity uncertainty.
   *
   * \return Current angular velocity covariance
   */
  virtual Eigen::Matrix3d getAngularVelocityUncertainty() const = 0;
};

#endif  // COLA2_NAV_EKF_BASE_H
