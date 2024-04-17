/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_SECTION_H_
#define COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_SECTION_H_

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
  double min_surge_velocity;
  double max_surge_velocity;
  double delta;
  double distance_to_max_velocity;
  double tolerance_z;
} SectionControllerConfig;

/**
 * \brief SectionController class
 * Computes the surge, heave and yaw motion to follow a control::Request using a
 * Line-of-Sight with Cross-Tracking-Error controller
 */
class SectionController
{
 protected:
  SectionControllerConfig config_;

 public:
  /**
   * Constructor. Requires an SectionControllerConfig structure
   */
  explicit SectionController(SectionControllerConfig);

  /**
   * Given the current control::State and the desired control::Request
   * computes the action to reach a waypoint
   */
  void compute(const control::State&, const control::Request&, control::State&, control::Feedback&,
               control::PointsList&);
  /**
   * Set configuration by means of a SectionControllerConfig struct
   */
  void setConfig(const SectionControllerConfig&);
};

#endif  // COLA2_CONTROL_INCLUDE_COLA2_CONTROL_CONTROLLERS_SECTION_H_
