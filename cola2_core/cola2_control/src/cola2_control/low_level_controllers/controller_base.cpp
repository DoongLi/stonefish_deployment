
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/controller_base.h>

IController::IController(std::string name) : name_(name)
{
}

void IController::reset()
{
}

double IController::compute(double, double, double)
{
  return 0.0;
}

bool IController::setParameters(std::map<std::string, double>)
{
  return false;
}
