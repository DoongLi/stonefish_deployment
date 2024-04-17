
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_

#include <cmath>
#include <iostream>
#include <map>
#include <string>

class IController
{
protected:
  std::string name_;

public:
  IController(std::string name);

  virtual void reset();

  virtual double compute(double time_in_sec, double setpoint, double feedback);

  virtual bool setParameters(std::map<std::string, double> params);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_ICONTROLLER_H_
