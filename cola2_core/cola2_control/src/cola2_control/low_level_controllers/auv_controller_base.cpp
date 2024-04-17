
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/auv_controller_base.h>

IAUVController::IAUVController(double period, int n_dof, int n_thrusters, int n_fins)
  : is_pose_controller_enable_(true)
  , is_velocity_controller_enable_(true)
  , is_thruster_allocator_enable_(true)
  , is_fin_allocator_enable_(true)
  , n_dof_(n_dof)
  , pose_merge_("pose_merge", "pose", period * 3)
  , pose_feedback_(n_dof, 0.0)
  , max_velocity_(n_dof, 0.2)
  , twist_merge_("twist_merge", "twist", period * 3)
  , twist_feedback_(n_dof, 0.0)
  , max_wrench_(n_dof, 100.0)
  , set_zero_velocity_depth_(1.0)
  , set_zero_velocity_priority_(5)
  , wrench_merge_("wrench_merge", "wrench", period * 3)
  , thruster_setpoints_(n_thrusters)
  , fin_setpoints_(n_fins)
{
}

void IAUVController::updatePoseRequest(Request req)
{
  pose_merge_.addRequest(req);
}

void IAUVController::updatePoseFeedback(std::vector<double> feedback)
{
  pose_feedback_ = feedback;
}

void IAUVController::updateTwistRequest(Request req)
{
  twist_merge_.addRequest(req);
}

void IAUVController::updateTwistFeedback(std::vector<double> feedback)
{
  twist_feedback_ = feedback;
}

void IAUVController::updateWrenchRequest(Request req)
{
  wrench_merge_.addRequest(req);
}

void IAUVController::addPolyParamToVector(std::vector<double> values,
                                          std::vector<std::map<std::string, double> >& params_vector)
{
  assert(values.size() == 3);
  std::map<std::string, double> param;
  param.insert(std::pair<std::string, double>("n_dof", 3.0));
  param.insert(std::pair<std::string, double>("0", values.at(0)));
  param.insert(std::pair<std::string, double>("1", values.at(1)));
  param.insert(std::pair<std::string, double>("2", values.at(2)));
  params_vector.push_back(param);
}

void IAUVController::addPIDParamToVector(std::vector<std::string> keys, std::vector<double> values,
                                         std::vector<std::map<std::string, double> >& params_vector)
{
  assert(keys.size() == values.size());
  std::map<std::string, double> param;
  for (unsigned int i = 0; i < keys.size(); i++)
  {
    param.insert(std::pair<std::string, double>(keys.at(i), values.at(i)));
  }
  // Add derivative_term_from_feedback
  param.insert(std::pair<std::string, double>("derivative_term_from_feedback", 1.0));
  params_vector.push_back(param);
}

void IAUVController::setMaxWrench(std::vector<double> max_wrench)
{
  assert(max_wrench.size() == max_wrench_.size());
  std::copy(max_wrench.begin(), max_wrench.end(), max_wrench_.begin());
}

void IAUVController::setMaxVelocity(std::vector<double> max_velocity)
{
  assert(max_velocity.size() == max_velocity_.size());
  std::copy(max_velocity.begin(), max_velocity.end(), max_velocity_.begin());
}

Request IAUVController::getMergedPose() const
{
  return merged_pose_;
}

Request IAUVController::getMergedTwist() const
{
  return merged_twist_;
}

Request IAUVController::getMergedWrench() const
{
  return merged_wrench_;
}

void IAUVController::setMergedWrench(const Request wrench)
{
  merged_wrench_ = wrench;
}

Eigen::VectorXd IAUVController::getThrusterSetpoints() const
{
  return thruster_setpoints_;
}

Eigen::VectorXd IAUVController::getFinSetpoints() const
{
  return fin_setpoints_;
}

void IAUVController::setPoseController(const bool is_enabled)
{
  is_pose_controller_enable_ = is_enabled;
}

void IAUVController::setVelocityController(const bool is_enabled)
{
  is_velocity_controller_enable_ = is_enabled;
}

void IAUVController::setThrusterAllocator(const bool is_enabled)
{
  is_thruster_allocator_enable_ = is_enabled;
}

void IAUVController::setFinAllocator(const bool is_enabled)
{
  is_fin_allocator_enable_ = is_enabled;
}

void IAUVController::setSetZeroVelocityDepth(const double depth)
{
  set_zero_velocity_depth_ = depth;
}

void IAUVController::setSetZeroVelocityPriority(const int priority)
{
  set_zero_velocity_priority_ = priority;
}

void IAUVController::setSetZeroVelocityAxes(const std::vector<bool>& axes)
{
  set_zero_velocity_axes_ = axes;
}

bool IAUVController::isPoseControllerEnable() const
{
  return is_pose_controller_enable_;
}

bool IAUVController::isVelocityControllerEnable() const
{
  return is_velocity_controller_enable_;
}

bool IAUVController::isThrusterAllocatorEnable() const
{
  return is_thruster_allocator_enable_;
}

bool IAUVController::isFinAllocatorEnable() const
{
  return is_fin_allocator_enable_;
}

bool IAUVController::getIsPoseControllerEnable() const
{
  return is_pose_controller_enable_;
}

bool IAUVController::getIsVelocityControllerEnable() const
{
  return is_velocity_controller_enable_;
}

bool IAUVController::getIsThrusterAllocatorEnable() const
{
  return is_thruster_allocator_enable_;
}

bool IAUVController::getIsFinAllocatorEnable() const
{
  return is_fin_allocator_enable_;
}

double IAUVController::getSetZeroVelocityDepth() const
{
  return set_zero_velocity_depth_;
}

int IAUVController::getSetZeroVelocityPriority() const
{
  return set_zero_velocity_priority_;
}

std::vector<bool> IAUVController::getSetZeroVelocityAxes() const
{
  return set_zero_velocity_axes_;
}

void IAUVController::setIsPoseControllerEnable(const bool& value)
{
  is_pose_controller_enable_ = value;
}

void IAUVController::setIsVelocityControllerEnable(const bool& value)
{
  is_velocity_controller_enable_ = value;
}

void IAUVController::setIsThrusterAllocatorEnable(const bool& value)
{
  is_thruster_allocator_enable_ = value;
}

void IAUVController::setIsFinAllocatorEnable(const bool& value)
{
  is_fin_allocator_enable_ = value;
}
