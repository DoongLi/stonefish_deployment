
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <cola2_control/low_level_controllers/controller_base.h>

class Poly : public IController
{
private:
  std::vector<double> setpoint_coefs_;

public:
  Poly(std::string name);

  void reset();

  double compute(double time_in_sec, double setpoint, double feedback);

  bool setParameters(std::map<std::string, double> params);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_POLY_H_
