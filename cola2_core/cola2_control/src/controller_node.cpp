/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/controllerConfig.h>
#include <cola2_control/low_level_controllers/only_thrusters_controller.h>
#include <cola2_control/ros_controller/auv_ros_controller_base.h>
#include <cola2_lib_ros/param_loader.h>
#include <dynamic_reconfigure/server.h>
#include <cola2_msgs/GoalDescriptor.h>
#include <ros/ros.h>
#include <string>

class OnlyThrustersROSController : public IAUVROSController
{
private:
  // AUV controller ptr.
  std::shared_ptr<OnlyThrustersController> auv_controller_;

  // Dynamic reconfigure parameters
  dynamic_reconfigure::Server<cola2_control::controllerConfig> param_server_;
  dynamic_reconfigure::Server<cola2_control::controllerConfig>::CallbackType f_;

public:
  /**
   * Class constructor
   * @param name Node name
   * @param frame_id Frame id in which messages must be published
   */
  OnlyThrustersROSController(const std::string name, const std::string frame_id) : IAUVROSController(name, frame_id)
  {
  }

  /**
   * Initialize the class OnlyThrustersROSController witha a std::shared_ptr<OnlyThrustersController>
   * @param auv_controller_ptr real C++ OnlyThrusters controller
   * @param period period time in seconds (e.g., 10Hz -> period = 0.1)
   */
  void init(std::shared_ptr<OnlyThrustersController> auv_controller_ptr, const double& period)
  {
    initBase(auv_controller_ptr, period);

    // Init pointer to AUV controller
    auv_controller_ = auv_controller_ptr;

    // Init dynamic reconfigure
    f_ = boost::bind(&OnlyThrustersROSController::setParams, this, _1, _2);
    param_server_.setCallback(f_);
  }

  /**
   * Callback for the dynamic reconfigure function to change the controller parameters.
   * @param config
   * @param level
   */
  void setParams(cola2_control::controllerConfig& config, uint32_t)
  {
    std::vector<std::map<std::string, double> > p_params;
    std::vector<std::string> keys = { "kp", "ti", "td", "i_limit", "fff" };
    std::vector<double> values1 = { config.p_surge_kp, config.p_surge_ti, config.p_surge_td, config.p_surge_i_limit,
                                    config.p_surge_fff };
    auv_controller_->addPIDParamToVector(keys, values1, p_params);
    std::vector<double> values2 = { config.p_sway_kp, config.p_sway_ti, config.p_sway_td, config.p_sway_i_limit,
                                    config.p_sway_fff };
    auv_controller_->addPIDParamToVector(keys, values2, p_params);
    std::vector<double> values3 = { config.p_heave_kp, config.p_heave_ti, config.p_heave_td, config.p_heave_i_limit,
                                    config.p_heave_fff };
    auv_controller_->addPIDParamToVector(keys, values3, p_params);
    std::vector<double> values4 = { config.p_roll_kp, config.p_roll_ti, config.p_roll_td, config.p_roll_i_limit,
                                    config.p_roll_fff };
    auv_controller_->addPIDParamToVector(keys, values4, p_params);
    std::vector<double> values5 = { config.p_pitch_kp, config.p_pitch_ti, config.p_pitch_td, config.p_pitch_i_limit,
                                    config.p_pitch_fff };
    auv_controller_->addPIDParamToVector(keys, values5, p_params);
    std::vector<double> values6 = { config.p_yaw_kp, config.p_yaw_ti, config.p_yaw_td, config.p_yaw_i_limit,
                                    config.p_yaw_fff };
    auv_controller_->addPIDParamToVector(keys, values6, p_params);

    std::vector<std::map<std::string, double> > t_params;
    std::vector<double> tvalues1 = { config.t_surge_kp, config.t_surge_ti, config.t_surge_td, config.t_surge_i_limit,
                                     config.t_surge_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues1, t_params);
    std::vector<double> tvalues2 = { config.t_sway_kp, config.t_sway_ti, config.t_sway_td, config.t_sway_i_limit,
                                     config.t_sway_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues2, t_params);
    std::vector<double> tvalues3 = { config.t_heave_kp, config.t_heave_ti, config.t_heave_td, config.t_heave_i_limit,
                                     config.t_heave_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues3, t_params);
    std::vector<double> tvalues4 = { config.t_roll_kp, config.t_roll_ti, config.t_roll_td, config.t_roll_i_limit,
                                     config.t_roll_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues4, t_params);
    std::vector<double> tvalues5 = { config.t_pitch_kp, config.t_pitch_ti, config.t_pitch_td, config.t_pitch_i_limit,
                                     config.t_pitch_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues5, t_params);
    std::vector<double> tvalues6 = { config.t_yaw_kp, config.t_yaw_ti, config.t_yaw_td, config.t_yaw_i_limit,
                                     config.t_yaw_fff };
    auv_controller_->addPIDParamToVector(keys, tvalues6, t_params);

    std::vector<std::map<std::string, double> > poly_params;
    std::cout << "Poly YAW: " << config.poly_yaw_B << "\n";
    std::vector<double> pvalues1 = { config.poly_surge_A, config.poly_surge_B, config.poly_surge_C };
    auv_controller_->addPolyParamToVector(pvalues1, poly_params);
    std::vector<double> pvalues2 = { config.poly_sway_A, config.poly_sway_B, config.poly_sway_C };
    auv_controller_->addPolyParamToVector(pvalues2, poly_params);
    std::vector<double> pvalues3 = { config.poly_heave_A, config.poly_heave_B, config.poly_heave_C };
    auv_controller_->addPolyParamToVector(pvalues3, poly_params);
    std::vector<double> pvalues4 = { config.poly_roll_A, config.poly_roll_B, config.poly_roll_C };
    auv_controller_->addPolyParamToVector(pvalues4, poly_params);
    std::vector<double> pvalues5 = { config.poly_pitch_A, config.poly_pitch_B, config.poly_pitch_C };
    auv_controller_->addPolyParamToVector(pvalues5, poly_params);
    std::vector<double> pvalues6 = { config.poly_yaw_A, config.poly_yaw_B, config.poly_yaw_C };
    auv_controller_->addPolyParamToVector(pvalues6, poly_params);

    std::vector<double> max_wrench = { config.max_wrench_X,    config.max_wrench_Y,     config.max_wrench_Z,
                                       config.max_wrench_Roll, config.max_wrench_Pitch, config.max_wrench_Yaw };
    std::vector<double> max_velocity = { config.max_velocity_x,    config.max_velocity_y,     config.max_velocity_z,
                                         config.max_velocity_roll, config.max_velocity_pitch, config.max_velocity_yaw };

    auv_controller_->setMaxVelocity(max_velocity);
    auv_controller_->setMaxWrench(max_wrench);

    double set_zero_velocity_depth = 1.0;
    std::vector<bool> set_zero_velocity_axes(6, true);
    cola2::ros::getParam("~set_zero_velocity_depth", set_zero_velocity_depth);
    cola2::ros::getParam("~set_zero_velocity_axes", set_zero_velocity_axes);
    if (set_zero_velocity_axes.size() != 6)
    {
      ROS_WARN("The set zero velocity axes parameter must be of size 6");
    }
    set_zero_velocity_axes.resize(6, true);
    auv_controller_->setSetZeroVelocityDepth(set_zero_velocity_depth);
    auv_controller_->setSetZeroVelocityPriority(cola2_msgs::GoalDescriptor::PRIORITY_SAFETY_LOW);
    auv_controller_->setSetZeroVelocityAxes(set_zero_velocity_axes);

    // Thruster allocator
    std::vector<std::vector<double> > poly_positive_v, poly_negative_v;
    std::vector<double> max_force_thruster_positive_v, max_force_thruster_negative_v;
    for (std::size_t i = 0; i < auv_controller_->getNumberofThrusters(); ++i)
    {
      std::vector<double> thruster_poly_positive, thruster_poly_negative;
      cola2::ros::getParamVector(std::string("~thruster_") + std::to_string(i + 1) + std::string("_poly_positive"),
                                 thruster_poly_positive);
      cola2::ros::getParamVector(std::string("~thruster_") + std::to_string(i + 1) + std::string("_poly_negative"),
                                 thruster_poly_negative);
      poly_positive_v.push_back(thruster_poly_positive);
      poly_negative_v.push_back(thruster_poly_negative);

      double max_force_positive, max_force_negative;
      cola2::ros::getParam(std::string("~thruster_") + std::to_string(i + 1) + std::string("_max_force_positive"),
                           max_force_positive);
      cola2::ros::getParam(std::string("~thruster_") + std::to_string(i + 1) + std::string("_max_force_negative"),
                           max_force_negative);
      max_force_thruster_positive_v.push_back(max_force_positive);
      max_force_thruster_negative_v.push_back(max_force_negative);
    }
    std::vector<double> tcm;
    cola2::ros::getParamVector("~TCM", tcm);
    auv_controller_->thruster_allocator_.setParams(max_force_thruster_positive_v, max_force_thruster_negative_v,
                                                   poly_positive_v, poly_negative_v, tcm);

    // Change Params in C++ Class
    auv_controller_->setControllerParams(p_params, t_params, poly_params);

    ROS_INFO_STREAM("config.enable_thrusters: " << config.enable_thrusters << std::endl);
    if (config.enable_thrusters)
    {
      ROS_INFO("Thruster enabled\n");
      auv_controller_->setThrusterAllocator(true);
    }
    else
    {
      ROS_INFO("Thruster disabled\n");
      auv_controller_->setThrusterAllocator(false);
    }

    std::cout << "Parameters changed!" << std::endl;
  }
};

int main(int argc, char** argv)
{
  // Init ROS controller
  ros::init(argc, argv, "controller");

  // TODO: period number of thrusters and number of DoF have to be read from rosparam server

  double period;
  int n_thrusters;
  cola2::ros::getParam("~period", period, 0.1);
  cola2::ros::getParam("~n_thrusters", n_thrusters, 5);

  // Init OnlyThrusters controller
  std::shared_ptr<OnlyThrustersController> auv_ctrl_ptr;
  auv_ctrl_ptr = std::shared_ptr<OnlyThrustersController>(new OnlyThrustersController(period, n_thrusters));

  // Init ROS node
  OnlyThrustersROSController ros_controller(cola2::ros::getUnresolvedNodeName(),
                                            cola2::ros::getNamespaceNoInitialDash() + "/base_link");

  // Initialize controller pointer into ROS node
  ros_controller.init(auv_ctrl_ptr, period);

  // Spin until architecture stops
  ros::spin();

  return 0;
}
