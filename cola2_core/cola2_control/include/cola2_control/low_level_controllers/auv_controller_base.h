
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <cola2_control/low_level_controllers/merge.h>
#include <cola2_control/low_level_controllers/request.h>

class IAUVController
{
private:
  // Are controllers enabled
  bool is_pose_controller_enable_;
  bool is_velocity_controller_enable_;
  bool is_thruster_allocator_enable_;
  bool is_fin_allocator_enable_;

protected:
  // DoF
  unsigned int n_dof_;

  // Pose
  Merge pose_merge_;
  std::vector<double> pose_feedback_;
  std::vector<double> max_velocity_;

  // Twist
  Merge twist_merge_;
  std::vector<double> twist_feedback_;
  std::vector<double> max_wrench_;

  // Set zero velocity
  double set_zero_velocity_depth_;
  int set_zero_velocity_priority_;
  std::vector<bool> set_zero_velocity_axes_;

  // Wrench
  Merge wrench_merge_;

  // Intermediate requests
  Request merged_pose_;
  Request merged_twist_;
  Request merged_wrench_;
  Eigen::VectorXd thruster_setpoints_;
  Eigen::VectorXd fin_setpoints_;

public:
  IAUVController(double period, int n_dof, int n_thrusters, int n_fins = 0);

  void updatePoseRequest(Request req);

  void updatePoseFeedback(std::vector<double> feedback);

  void updateTwistRequest(Request req);

  void updateTwistFeedback(std::vector<double> feedback);

  void updateWrenchRequest(Request req);

  void addPolyParamToVector(std::vector<double> values, std::vector<std::map<std::string, double> >& params_vector);

  void addPIDParamToVector(std::vector<std::string> keys, std::vector<double> values,
                           std::vector<std::map<std::string, double> >& params_vector);

  void setMaxWrench(std::vector<double> max_wrench);

  void setMaxVelocity(std::vector<double> max_velocity);

  Request getMergedPose() const;

  Request getMergedTwist() const;

  Request getMergedWrench() const;

  void setMergedWrench(const Request wrench);

  Eigen::VectorXd getThrusterSetpoints() const;

  Eigen::VectorXd getFinSetpoints() const;

  void setPoseController(const bool is_enabled);

  void setVelocityController(const bool is_enabled);

  void setThrusterAllocator(const bool is_enabled);

  void setFinAllocator(const bool is_enabled);

  void setSetZeroVelocityDepth(const double depth);

  void setSetZeroVelocityPriority(const int priority);

  void setSetZeroVelocityAxes(const std::vector<bool>& priority);

  bool isPoseControllerEnable() const;

  bool isVelocityControllerEnable() const;

  bool isThrusterAllocatorEnable() const;

  bool isFinAllocatorEnable() const;

  bool getIsPoseControllerEnable() const;

  bool getIsVelocityControllerEnable() const;

  bool getIsThrusterAllocatorEnable() const;

  bool getIsFinAllocatorEnable() const;

  double getSetZeroVelocityDepth() const;

  int getSetZeroVelocityPriority() const;

  std::vector<bool> getSetZeroVelocityAxes() const;

  void setIsPoseControllerEnable(const bool& value);

  void setIsVelocityControllerEnable(const bool& value);

  void setIsThrusterAllocatorEnable(const bool& value);

  void setIsFinAllocatorEnable(const bool& value);

  virtual void iteration(double current_time) = 0;

  virtual void reset() = 0;

  virtual void computeThrusterAllocator() = 0;

  virtual unsigned int getNumberofThrusters() const = 0;
};

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVCONTROLLER_H_
