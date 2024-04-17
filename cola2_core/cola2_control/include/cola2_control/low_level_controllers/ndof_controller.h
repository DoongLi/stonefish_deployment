
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_

#include <cola2_control/low_level_controllers/controller_base.h>
#include <cola2_control/low_level_controllers/request.h>
#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <vector>

class NDofController
{
private:
  std::vector<std::shared_ptr<IController> > controllers_;
  unsigned int n_dof_;

public:
  NDofController(const unsigned int n_dof = 6);

  void addController(std::shared_ptr<IController> controller);

  void setControllerParams(std::vector<std::map<std::string, double> > params);

  void reset();

  std::vector<double> compute(double time_in_sec, Request req, std::vector<double> feedback);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_NDOFCONTROLLER_H_
