
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_

#include <map>
#include <string>

#include <cola2_control/low_level_controllers/controller_base.h>
#include <cola2_lib/utils/saturate.h>

class PID : public IController
{
private:
  double kp_;
  double ti_;
  double td_;
  double i_limit_;
  double fff_;
  double time_old_;
  double feedback_old_;
  double error_old_;
  double eik_old_;
  bool derivative_term_from_feedback_;

  // for filtering purposes
  double edotk_old_;

public:
  PID(std::string name);

  void reset();

  double compute(double time_in_sec, double setpoint, double feedback);

  bool setParameters(std::map<std::string, double> params);
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_PID_H_
