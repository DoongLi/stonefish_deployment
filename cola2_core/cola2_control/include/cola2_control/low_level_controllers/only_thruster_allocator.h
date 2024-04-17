
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __ONLYTHRUSTERALLOCATOR_CLASS__
#define __ONLYTHRUSTERALLOCATOR_CLASS__

#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_control/low_level_controllers/request.h>
#include <cola2_lib/utils/saturate.h>
#include <algorithm>
#include <cassert>
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

class OnlyThrusterAllocator
{
protected:
  std::size_t n_thrusters_;
  std::vector<double> max_force_thruster_positive_v_;
  std::vector<double> max_force_thruster_negative_v_;
  Eigen::MatrixXd tcm_inv_;
  std::vector<Poly> poly_positive_v_;
  std::vector<Poly> poly_negative_v_;
  bool is_init_;

  /**
   * From newtons or newtons meter to setpoints for each thruster
   * @param thruster_forces Forces for each thruster
   * @return setpoint for each thruster
   */
  Eigen::VectorXd forceToSetpoint(Eigen::VectorXd thruster_forces);

  /**
   * Combine wrench request to acheive a force per thruster that preserves the most important DoFs
   * @param wrench Force per DoF
   * @return Force per thruster
   */
  Eigen::VectorXd wrenchToThrusterForces(const Eigen::VectorXd& wrench);

public:
  /**
   * Class constructor
   * @param n_thrusters number of thrusters in the vehicle
   */
  OnlyThrusterAllocator(unsigned int n_thrusters);

  ~OnlyThrusterAllocator();

  void setParams(const std::vector<double>& max_force_thruster_positive_v,
                 const std::vector<double>& max_force_thruster_negative_v,
                 const std::vector<std::vector<double> >& poly_positive_v,
                 const std::vector<std::vector<double> >& poly_negative_v, const std::vector<double>& tcm_values);

  /**
   * Computes the setpoint for each thrusters taking into account the force + torque (wrench) to be
   * achieved by the vehicle
   * @param wrench Desired force + troque (6DoFs)
   * @return setpoint for each thruster
   */
  Eigen::VectorXd compute(const Request& wrench);
};

#endif  // __ONLYTHRUSTERALLOCATOR_CLASS__
