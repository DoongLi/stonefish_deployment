
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/ndof_controller.h>

NDofController::NDofController(const unsigned int n_dof) : n_dof_(n_dof)
{
}

void NDofController::addController(std::shared_ptr<IController> controller)
{
  assert(controllers_.size() < n_dof_);
  controllers_.push_back(controller);
}

void NDofController::setControllerParams(std::vector<std::map<std::string, double> > params)
{
  assert(params.size() == n_dof_);
  assert(controllers_.size() == n_dof_);
  for (unsigned int i = 0; i < n_dof_; i++)
  {
    controllers_.at(i)->setParameters(params.at(i));
  }
}

void NDofController::reset()
{
  for (unsigned int i = 0; i < n_dof_; i++)
  {
    controllers_.at(i)->reset();
  }
}

std::vector<double> NDofController::compute(double time_in_sec, Request req, std::vector<double> feedback)
{
  assert(controllers_.size() == n_dof_);
  std::vector<double> ret;
  std::vector<double> setpoint = req.getValues();
  std::vector<bool> disable_axis = req.getDisabledAxis();

  for (unsigned int i = 0; i < n_dof_; i++)
  {
    if (!disable_axis.at(i))
    {
      ret.push_back(controllers_.at(i)->compute(time_in_sec, setpoint.at(i), feedback.at(i)));
    }
    else
    {
      controllers_.at(i)->compute(time_in_sec, setpoint.at(i), feedback.at(i));
      controllers_.at(i)->reset();
      ret.push_back(0.0);
    }
  }
  return ret;
}
