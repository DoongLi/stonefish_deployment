
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef __OnlyThrusters_CONTROLLER__
#define __OnlyThrusters_CONTROLLER__

#include <cola2_control/low_level_controllers/auv_controller_base.h>
#include <cola2_control/low_level_controllers/merge.h>
#include <cola2_control/low_level_controllers/ndof_controller.h>
#include <cola2_control/low_level_controllers/only_thruster_allocator.h>
#include <cola2_control/low_level_controllers/pid.h>
#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_lib/utils/angles.h>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <memory>

class OnlyThrustersController : public IAUVController
{
private:
  //  Number of thrusters
  unsigned int n_thrusters_;

  // Pose PID controller
  std::shared_ptr<PID> p_surge_;
  std::shared_ptr<PID> p_sway_;
  std::shared_ptr<PID> p_heave_;
  std::shared_ptr<PID> p_roll_;
  std::shared_ptr<PID> p_pitch_;
  std::shared_ptr<PID> p_yaw_;
  NDofController pose_controller_;

  // Twist PID Controller
  std::shared_ptr<PID> t_surge_;
  std::shared_ptr<PID> t_sway_;
  std::shared_ptr<PID> t_heave_;
  std::shared_ptr<PID> t_roll_;
  std::shared_ptr<PID> t_pitch_;
  std::shared_ptr<PID> t_yaw_;
  NDofController twist_controller_;

  // Twist Poly controller
  std::shared_ptr<Poly> tp_surge_;
  std::shared_ptr<Poly> tp_sway_;
  std::shared_ptr<Poly> tp_heave_;
  std::shared_ptr<Poly> tp_roll_;
  std::shared_ptr<Poly> tp_pitch_;
  std::shared_ptr<Poly> tp_yaw_;
  NDofController twist_poly_controller_;

public:
  /**
   * Class constructor.
   * @param period Execution period in seconds
   * @param n_thrusters Number of thrusters in the vehicle
   */
  OnlyThrustersController(double period, unsigned int n_thrusters);

  /**
   * Initialize PID pose controller.
   */
  void initPoseController();

  /**
   * Initialize PID twist controller.
   */
  void initTwistController();

  /**
   * Initialize Polinomic twist controller.
   */
  void initTwistPolyController();

  /**
   * Set the parameters for each controller: PID pose, PID twist and Polinomial twist
   * @param p_params
   * @param t_params
   * @param poly_params
   */
  void setControllerParams(const std::vector<std::map<std::string, double> > p_params,
                           const std::vector<std::map<std::string, double> > t_params,
                           const std::vector<std::map<std::string, double> > poly_params);

  /**
   * Resets PID pose, PID twist and Polinomial twist controllers
   */
  void reset();

  /**
   * Main class that merges pose request and applies the PID pose controller, then merges twist requests
   * (including the output of the pose controller) and apply both PID and Poly Twist controllers. The output of both
   * is combined (by adding them) and normalized. The Wrench to be applied is obtained at the end.
   * @param current_time absolute time in which iteration is called to be able to measure the elapsed time.
   */
  void iteration(double current_time);

  /**
   * Compute error in pose
   * @param setpoint desired position + orientation
   * @param feedback current positio + orientation
   * @return error in meters and radians
   */
  std::vector<double> computeError(const std::vector<double> setpoint, const std::vector<double> feedback);

  /**
   * Computes the error in position in the plane X, Y.
   * WARNING! It does not take into account Roll and Pitch
   * @param setpoint desired position + orientation
   * @param feedback current positio + orientation
   * @return error in meters
   */
  void poseError(const std::vector<double> setpoint, const std::vector<double> feedback, std::vector<double>& error);

  /**
   * Computes the setpoint for each thruster according the Wrench calculated at the iteration method.
   * It uses a thruste allocator task defined in a another file.
   */
  void computeThrusterAllocator();

  OnlyThrusterAllocator thruster_allocator_;

  /**
   * @return The number of thrusters.
   */
  unsigned int getNumberofThrusters() const;
};

#endif  //__OnlyThrusters_CONTROLLER__
