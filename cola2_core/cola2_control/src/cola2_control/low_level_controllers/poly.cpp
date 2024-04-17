
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/poly.h>

Poly::Poly(std::string name) : IController(name), setpoint_coefs_()
{
}

void Poly::reset()
{
}

double Poly::compute(double, double setpoint, double)
{
  double tau = 0.0;
  bool negative_setpoint = false;
  if (setpoint < 0)
    negative_setpoint = true;

  for (unsigned int i = 0; i < setpoint_coefs_.size(); i++)
  {
    tau = tau + std::pow(static_cast<double>(std::fabs(setpoint)), static_cast<double>(i)) * setpoint_coefs_.at(i);
  }
  if (negative_setpoint)
    tau = -1.0 * tau;

  // Return non-saturated tau
  return tau;
}

bool Poly::setParameters(std::map<std::string, double> params)
{
  // std::cout << "Set params for " << name_ << ": " << static_cast<int>(params["1"]) << "\n";
  setpoint_coefs_.clear();
  try
  {
    unsigned int n_dof = static_cast<int>(params["n_dof"]);
    // std::cout << "poly n_dof: " << n_dof << "\n";
    for (unsigned int i = 0; i < n_dof; i++)
    {
      std::ostringstream s;
      s << i;
      const std::string i_as_string(s.str());
      setpoint_coefs_.push_back(static_cast<double>(params[i_as_string]));
      // std::cout << "add param: " << double( params[i_as_string] ) << "\n";
    }
  }
  catch (...)
  {
    return false;
  }
  // std::cout << "Poly initialized!\n";
  return true;
}
