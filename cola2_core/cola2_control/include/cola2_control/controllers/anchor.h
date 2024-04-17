/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_ANCHOR_H_
#define COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_ANCHOR_H_

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
  double kp;
  double radius;
  double min_surge_velocity;
  double max_surge_velocity;
} AnchorControllerConfig;

/**
 * \brief COLA2 Anchor controller
 * To keep a non holonomic vehicle in a point
 */
class AnchorController
{
 protected:
  AnchorControllerConfig config_;

 public:
  /**
   * Constructor. Requires an AnchorControllerConfig structure
   */
  explicit AnchorController(AnchorControllerConfig);

  /**
   * Given the current control::State and the desired control::Request
   * computes the action to keep the vehicle anchored
   */
  void compute(const control::State&, const control::Request&, control::State&, control::Feedback&,
               control::PointsList&);

  /**
   * Set configuration by means of a AnchorControllerConfig struct
   */
  void setConfig(const AnchorControllerConfig&);
};

#endif  // COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_ANCHOR_H_
