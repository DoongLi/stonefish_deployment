/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_HOLONOMIC_KEEP_POSITION_H_
#define COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_HOLONOMIC_KEEP_POSITION_H_

#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/angles.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

typedef struct
{
  double no_params;
} HolonomicKeepPositionControllerConfig;

/**
 * \brief HolonomicGotoController class
 * Computes the surge, sway, heave and yaw motion to reach a waypoint
 */
class HolonomicKeepPositionController
{
 protected:
  HolonomicKeepPositionControllerConfig config_;

 public:
  /**
   * Constructor. Requires an HolonomicKeepPositionControllerConfig structure
   */
  explicit HolonomicKeepPositionController(HolonomicKeepPositionControllerConfig);

  /**
   * Given the current control::State and the desired control::Request
   * computes the action to reach a waypoint
   */
  void compute(const control::State&, const control::Request&, control::State&, control::Feedback&,
               control::PointsList&);

  /**
   * Set configuration by means of a HolonomicKeepPositionControllerConfig struct
   */
  void setConfig(const HolonomicKeepPositionControllerConfig&);
};

#endif  // COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_HOLONOMIC_KEEP_POSITION_H_
