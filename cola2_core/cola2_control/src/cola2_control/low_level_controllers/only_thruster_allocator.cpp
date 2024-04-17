/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/only_thruster_allocator.h>

OnlyThrusterAllocator::OnlyThrusterAllocator(unsigned int n_thrusters)
  : n_thrusters_(static_cast<std::size_t>(n_thrusters))
  , max_force_thruster_positive_v_(n_thrusters_, 0.0)
  , max_force_thruster_negative_v_(n_thrusters_, 0.0)
  , poly_positive_v_(n_thrusters_, Poly("thruster_allocator_positive_poly"))
  , poly_negative_v_(n_thrusters_, Poly("thruster_allocator_negative_poly"))
  , is_init_(false)
{
}

OnlyThrusterAllocator::~OnlyThrusterAllocator()
{
}

void OnlyThrusterAllocator::setParams(const std::vector<double>& max_force_thruster_positive_v,
                                      const std::vector<double>& max_force_thruster_negative_v,
                                      const std::vector<std::vector<double> >& poly_positive_v,
                                      const std::vector<std::vector<double> >& poly_negative_v,
                                      const std::vector<double>& tcm_values)
{
  // std::cout << "OnlyThrusterAllocator set params\n";

  // Check sizes
  assert(max_force_thruster_positive_v.size() == n_thrusters_);
  assert(max_force_thruster_negative_v.size() == n_thrusters_);
  assert(tcm_values.size() == 6 * n_thrusters_);

  // Copy the rest
  max_force_thruster_positive_v_ = max_force_thruster_positive_v;
  max_force_thruster_negative_v_ = max_force_thruster_negative_v;

  // Positive and negative polys
  for (std::size_t i = 0; i < poly_positive_v.size(); ++i)
  {
    std::map<std::string, double> params_positive;
    params_positive["n_dof"] = poly_positive_v[i].size();
    for (std::size_t j = 0; j < poly_positive_v[i].size(); ++j)
    {
      params_positive[std::to_string(j)] = poly_positive_v[i][j];
    }
    poly_positive_v_[i].setParameters(params_positive);
  }
  for (std::size_t i = 0; i < poly_negative_v.size(); ++i)
  {
    std::map<std::string, double> params_negative;
    params_negative["n_dof"] = poly_negative_v[i].size();
    for (std::size_t j = 0; j < poly_negative_v[i].size(); ++j)
    {
      params_negative[std::to_string(j)] = poly_negative_v[i][j];
    }
    poly_negative_v_[i].setParameters(params_negative);
  }

  // TCM inverse
  // std::cout << "tcm_values.size(): " << tcm_values.size() << "\n";
  Eigen::MatrixXd tcm(6, n_thrusters_);
  for (std::size_t i = 0; i < 6; ++i)
  {
    for (std::size_t j = 0; j < n_thrusters_; ++j)
    {
      tcm(i, j) = tcm_values.at(i * n_thrusters_ + j);
    }
  }

  // std::cout << "TCM:\n" << tcm << "\n";
  tcm_inv_ = (tcm.transpose() * tcm).inverse() * tcm.transpose();
  // std::cout << "TCM inv:\n" << tcm_inv_ << "\n";

  // std::cout << "OnlyThrusterAllocator initialized!\n";
  is_init_ = true;
}

Eigen::VectorXd OnlyThrusterAllocator::compute(const Request& wrench_req)
{
  if (!is_init_)
    return Eigen::VectorXd::Zero(n_thrusters_);

  // Take wrench request if disabled false, otherwise, take 0.0
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  for (std::size_t i = 0; i < wrench_req.getDisabledAxis().size(); ++i)
  {
    if (!wrench_req.getDisabledAxis().at(i))
    {
      wrench[i] = wrench_req.getValues().at(i);
    }
  }
  std::cout << "wrench: \n" << wrench << "\n";

  // From wrench to forces per thruster
  Eigen::VectorXd force_per_thruster = wrenchToThrusterForces(wrench);
  std::cout << "force_per_thruster: \n" << force_per_thruster << "\n";

  // Force to setpoint
  Eigen::VectorXd setpoint = forceToSetpoint(force_per_thruster);
  std::cout << "setpoint: \n" << setpoint << "\n";

  // Publish
  return setpoint;
}

Eigen::VectorXd OnlyThrusterAllocator::forceToSetpoint(Eigen::VectorXd thruster_forces)
{
  Eigen::VectorXd setpoints = Eigen::VectorXd::Zero(thruster_forces.size());

  // Compute newtons or newtons meter to setpoints for each thruster
  for (std::size_t i = 0; i < static_cast<std::size_t>(thruster_forces.size()); ++i)
  {
    if (thruster_forces[i] > 0.0)
    {
      thruster_forces[i] = cola2::utils::saturate(thruster_forces[i], max_force_thruster_positive_v_[i]);
      setpoints[i] = poly_positive_v_[i].compute(0.0, thruster_forces[i], 0.0);
    }
    else if (thruster_forces[i] < 0.0)
    {
      thruster_forces[i] = cola2::utils::saturate(thruster_forces[i], max_force_thruster_negative_v_[i]);
      setpoints[i] = -poly_negative_v_[i].compute(0.0, -thruster_forces[i], 0.0);
    }

    setpoints[i] = cola2::utils::saturate(setpoints[i], 1.0);
  }

  return setpoints;
}

Eigen::VectorXd OnlyThrusterAllocator::wrenchToThrusterForces(const Eigen::VectorXd& wrench)
{
  // Define order to specify which DoFs are the most important
  const std::vector<std::size_t> order = { 5, 0, 1, 2, 3, 4 };

  // Initial setpoint
  Eigen::VectorXd setpoint = Eigen::VectorXd::Zero(n_thrusters_);

  // Add DoFs setpoints in order
  for (const auto& i : order)
  {
    Eigen::VectorXd isolated_wrench = Eigen::VectorXd::Zero(6);
    isolated_wrench[i] = wrench[i];
    Eigen::VectorXd setpoint_to_add = tcm_inv_ * isolated_wrench;

    double min_factor = 1.0;
    for (std::size_t j = 0; j < n_thrusters_; ++j)
    {
      if (setpoint_to_add[j] > 0.0)
      {
        if (setpoint[j] + setpoint_to_add[j] > max_force_thruster_positive_v_[j])
        {
          double factor = (max_force_thruster_positive_v_[j] - setpoint[j]) / setpoint_to_add[j];
          // std::cout << "Factor = " << factor << std::endl;
          if (factor < min_factor)
            min_factor = factor;
        }
      }
      else if (setpoint_to_add[j] < 0.0)
      {
        if (setpoint[j] + setpoint_to_add[j] < -max_force_thruster_negative_v_[j])
        {
          double factor = (setpoint[j] + max_force_thruster_negative_v_[j]) / (-setpoint_to_add[j]);
          // std::cout << "Factor = " << factor << std::endl;
          if (factor < min_factor)
            min_factor = factor;
        }
      }
    }

    setpoint += min_factor * setpoint_to_add;
  }
  return setpoint;
}
