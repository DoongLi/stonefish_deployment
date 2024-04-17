/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/poly.h>
#include <cola2_lib/utils/angles.h>
#include <cola2_lib/utils/saturate.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/setpoints_selector.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/BodyForceReq.h>
#include <cola2_msgs/Setpoints.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define STD_GRAVITY 9.80665  // Gravity constant

// Typedefs for common vector and matrices used in this node
namespace Eigen
{
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
}  // namespace Eigen

// Helper function to read matrices from the param server
void getParamVector3d(const std::string& tname, Eigen::Vector3d& v)
{
  std::vector<double> vec;
  cola2::ros::getParam(tname, vec);
  assert(vec.size() == 3);
  for (std::size_t i = 0; i < 3; ++i)
    v(i) = vec[i];
}
void getParamVector6d(const std::string& tname, Eigen::Vector6d& v)
{
  std::vector<double> vec;
  cola2::ros::getParam(tname, vec);
  assert(vec.size() == 6);
  for (std::size_t i = 0; i < 6; ++i)
    v(i) = vec[i];
}
void getParamMatrix3d(const std::string& tname, Eigen::Matrix3d& v)
{
  std::vector<double> vec;
  cola2::ros::getParam(tname, vec);
  assert(vec.size() == 9);
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
    {
      v(i, j) = vec[i * 3 + j];  // First row gets filled first
    }
  }
}
void getParamMatrixXd(const std::string& tname, Eigen::MatrixXd& v, std::size_t rows)
{
  std::vector<double> vec;
  cola2::ros::getParam(tname, vec);
  assert(!vec.empty());
  assert(vec.size() % rows == 0);
  const std::size_t cols = vec.size() / rows;
  v = Eigen::MatrixXd::Zero(rows, cols);
  for (std::size_t i = 0; i < rows; ++i)
  {
    for (std::size_t j = 0; j < cols; ++j)
    {
      v(i, j) = vec[i * cols + j];
    }
  }
}

// Make the skew-symmetric matrix representation of the vector for cross-product
Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& x)
{
  Eigen::Matrix3d x_hat;
  x_hat << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  return x_hat;
}

// Bisection Poly solver. The polynomial must be monotonic between x1 and x2
double bisectionPolySolver(Poly& poly, double x1, double x2, const double ytarget)
{
  const double y1 = poly.compute(0.0, x1, 0.0);  // Poly::compute is not const...
  const double y2 = poly.compute(0.0, x2, 0.0);
  if (y2 < y1)
  {
    std::swap(x1, x2);
  }

  double xm;
  for (std::size_t i = 0; i < 30; ++i)
  {
    xm = 0.5 * (x1 + x2);
    const double ym = poly.compute(0.0, xm, 0.0);
    if (ym < ytarget)
    {
      x1 = xm;
    }
    else
    {
      x2 = xm;
    }
  }
  return xm;
}

/**
 * @brief Simulates the dynamics of an AUV from thrusters setpoint and fins angles.
 */
class Dynamics
{
protected:
  // ROS variables
  ros::NodeHandle nh_;
  ros::Publisher pub_odom_, pub_odom_gazebo_;
  ros::Subscriber sub_force_, sub_thrusters_, sub_fins_, sub_current_, sub_pose_overwrite_;
  ros::ServiceServer srv_reload_params_;
  ros::ServiceClient srv_publish_params_;
  cola2::ros::DiagnosticHelper thrusters_diag_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer timer_check_actuators_;

  // Internal state
  Eigen::Vector6d p_, p_dot_;
  Eigen::Vector6d v_, v_dot_;
  Eigen::Matrix6d M_, IM_;
  Eigen::VectorXd u_, old_u_;
  Eigen::Vector2d f_, old_f_;
  Eigen::Vector3d current_;
  cola2_msgs::BodyForceReq force_;
  double last_thrusters_setpoint_sec_, last_fins_setpoint_sec_;
  cola2::ros::SetpointsSelector setpoints_selector_;

  struct Config
  {
    // Frames
    std::string world_frame_id;

    // Period and rate
    double period;
    double rate;

    // Topics
    std::string fins_topic;

    // Initial pose and velocity
    Eigen::Vector6d p0;
    Eigen::Vector6d v0;

    // Vehicle properties
    double mass;
    double buoyancy;
    double radius;
    double water_density;
    Eigen::Matrix3d tensor;
    Eigen::Vector3d buoyancy_center;
    Eigen::Vector6d damping;
    Eigen::Vector6d quadratic_damping;

    // Thrusters
    std::size_t thrusters_num;
    double thrusters_tau;
    Eigen::MatrixXd thrusters_matrix;
    std::vector<double> thrusters_limiter;
    double thrusters_max_step;
    bool thrusters_symmetric;
    std::vector<double> thrusters_max_force_positive, thrusters_max_force_negative;
    std::vector<Poly> thrusters_poly_positive, thrusters_poly_negative;

    // Fins
    double a_fins;
    double k_cd_fins;
    double k_cl_fins;
    double max_fins_angle;

    // Force
    bool use_force_topic;
  } config_;

  // Methods
  void initializeMassMatrix();
  void thrustersCallback(const cola2_msgs::Setpoints&);
  void finsCallback(const cola2_msgs::Setpoints&);
  void forceCallback(const cola2_msgs::BodyForceReq&);
  void currentCallback(const geometry_msgs::Vector3Stamped&);
  void poseOverwriteCallback(const nav_msgs::Odometry&);
  void checkActuatorsCallback(const ros::TimerEvent&);
  bool reloadConfigServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  Eigen::Vector6d computeThrustersForce(const Eigen::VectorXd&);
  Eigen::Vector6d computeFinsForce(const Eigen::Vector2d&, const Eigen::Vector6d&);
  Eigen::Vector6d gravityAndBuoyancyForce(const Eigen::Vector6d&);
  Eigen::Matrix6d dampingMatrix(const Eigen::Vector6d&);
  Eigen::Matrix6d coriolisMatrix(const Eigen::Vector6d&);
  Eigen::Vector6d kinematics(const Eigen::Vector6d&, const Eigen::Vector6d&);
  Eigen::Vector6d inverseDynamic(const Eigen::Vector6d&, const Eigen::Vector6d&, const Eigen::VectorXd&,
                                 const Eigen::Vector2d&);
  void getStaticConfig();
  void getVariableConfig();
  void publishOdometry();

public:
  Dynamics();
  void iterate();
  double getRate() const;
};

/**
 * @brief Dynamics constructor. Loads config, calls initialization method and creates the ROS interface.
 */
Dynamics::Dynamics() : nh_("~"), thrusters_diag_(nh_, "thrusters", cola2::ros::getUnresolvedNodeName()), setpoints_selector_(nh_)
{
  // Load parameters
  getStaticConfig();
  getVariableConfig();

  // Initialize internal data
  p_ = config_.p0;                                    // Initial position
  p_dot_ = Eigen::Vector6d::Zero();                   // Derivative of initial position is zero
  v_ = config_.v0;                                    // Initial velocity
  v_dot_ = Eigen::Vector6d::Zero();                   // Derivative of initial velocity is zero
  current_ = Eigen::Vector3d::Zero();                 // Initial water current is zero
  force_ = cola2_msgs::BodyForceReq();                // Initial force is zero
  u_ = Eigen::VectorXd::Zero(config_.thrusters_num);  // Initial thrusters setpoint is zero
  old_u_ = u_;                                        // Previous thrusters setpoint is zero
  f_ = Eigen::Vector2d::Zero();                       // Initial fins setpoint is zero
  old_f_ = f_;                                        // Previous fins setpoint is zero

  // Initialize mass matrix
  initializeMassMatrix();

  // Initialize last setpoints times
  last_thrusters_setpoint_sec_ = ros::Time::now().toSec();
  last_fins_setpoint_sec_ = last_thrusters_setpoint_sec_;

  // Publishers
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odometry", 2);
  pub_odom_gazebo_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 2);

  // Subscribers
  sub_thrusters_ = nh_.subscribe(cola2::ros::getNamespace() + "/controller/thruster_setpoints", 10,
                                 &Dynamics::thrustersCallback, this);
  if (!config_.fins_topic.empty())
  {
    sub_fins_ = nh_.subscribe(config_.fins_topic, 10, &Dynamics::finsCallback, this);
  }
  sub_force_ = nh_.subscribe(cola2::ros::getNamespace() + "/controller/merged_body_force_req", 10,
                             &Dynamics::forceCallback, this);
  sub_current_ = nh_.subscribe("current", 10, &Dynamics::currentCallback, this);
  sub_pose_overwrite_ = nh_.subscribe("pose_overwrite", 10, &Dynamics::poseOverwriteCallback, this);

  // Timers
  timer_check_actuators_ = nh_.createTimer(ros::Duration(1.0), &Dynamics::checkActuatorsCallback, this);

  // Service client to publish parameters
  std::string publish_params_srv_name = cola2::ros::getNamespace() + "/param_logger/publish_params";
  srv_publish_params_ = nh_.serviceClient<std_srvs::Trigger>(publish_params_srv_name);
  while (ros::ok())
  {
    if (srv_publish_params_.waitForExistence(ros::Duration(5.0)))
    {
      break;
    }
    ROS_INFO_STREAM("Waiting for client to service " << publish_params_srv_name);
  }

  // Services
  srv_reload_params_ = nh_.advertiseService("reload_params", &Dynamics::reloadConfigServiceCallback, this);

  thrusters_diag_.setEnabled(true);
  ROS_INFO_STREAM("Initialized");
}

/**
 * @brief Initialize mass matrix.
 */
void Dynamics::initializeMassMatrix()
{
  // Mass and inertia matrix of the rigid body when computed from the center of gravity
  // Mrb=[m,      0,      0,      0,      0,      0,
  //      0,      m,      0,      0,      0,      0,
  //      0,      0,      m,      0,      0,      0,
  //      0,      0,      0,    Ixx,    Ixy,    Ixz,
  //      0,      0,      0,    Iyx,    Iyy,    Iyz,
  //      0,      0,      0,    Izx,    Izy,    Izz]
  Eigen::Matrix6d Mrb = Eigen::Matrix6d::Zero();
  Mrb.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config_.mass;
  Mrb.block<3, 3>(3, 3) = config_.tensor;

  // Added mass matrix. We estimate to be half of the vehicle weight
  // Ma=[m/2,      0,      0,      0,      0,      0,
  //       0,    m/2,      0,      0,      0,      0,
  //       0,      0,    m/2,      0,      0,      0,
  //       0,      0,      0,      0,      0,      0,
  //       0,      0,      0,      0,      0,      0,
  //       0,      0,      0,      0,      0,      0]
  Eigen::Matrix6d Ma = Eigen::Matrix6d::Zero();
  Ma.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config_.mass * 0.5;

  // Total mass matrix: Mrb + Ma
  M_ = Mrb + Ma;
  IM_ = M_.inverse();
}

/**
 * @brief Thruster callback, input in range [-1, 1].
 */
void Dynamics::thrustersCallback(const cola2_msgs::Setpoints& msg)
{
  // Check thruster setpoints size
  if (msg.setpoints.size() != config_.thrusters_num)
  {
    ROS_ERROR_STREAM("Invalid thrusters setpoint length");
    return;
  }

  // Check the message with the setpoints selector
  if (!setpoints_selector_.acceptSetpoints(msg))
  {
    ROS_ERROR_THROTTLE(5, "The setpoints selector is rejecting messages");
    return;
  }

  // Process setpoints
  for (std::size_t i = 0; i < config_.thrusters_num; ++i)
  {
    // The following piece of code mimics the thrusters driver and response
    double setpoint = msg.setpoints[i];

    // Limiter
    setpoint = std::min(setpoint, config_.thrusters_limiter[i]);
    setpoint = std::max(setpoint, -config_.thrusters_limiter[i]);

    // Derivative step filter
    double max_allowed = u_(i) + config_.thrusters_max_step;
    double min_allowed = u_(i) - config_.thrusters_max_step;
    if (!config_.thrusters_symmetric)
    {
      if (u_(i) > 0.0)
      {
        min_allowed = -config_.thrusters_max_step;
      }
      else
      {
        max_allowed = +config_.thrusters_max_step;
      }
    }
    setpoint = std::min(setpoint, max_allowed);
    setpoint = std::max(setpoint, min_allowed);

    // Store setpoint
    u_(i) = setpoint;
  }
  last_thrusters_setpoint_sec_ = ros::Time::now().toSec();
  thrusters_diag_.reportValidData();
}

/**
 * @brief Fins callback, input in range [-max_angle, max_angle].
 */
void Dynamics::finsCallback(const cola2_msgs::Setpoints& msg)
{
  if (msg.setpoints.size() != 2)
  {
    ROS_ERROR_STREAM("Invalid fins setpoint length");
    return;
  }
  for (std::size_t i = 0; i < 2; ++i)
  {
    f_(i) = cola2::utils::saturate(msg.setpoints[i], config_.max_fins_angle);
  }
  last_fins_setpoint_sec_ = ros::Time::now().toSec();
}

/**
 * @brief Force callback.
 */
void Dynamics::forceCallback(const cola2_msgs::BodyForceReq& msg)
{
  force_ = msg;
}

/**
 * @brief Current callback.
 */
void Dynamics::currentCallback(const geometry_msgs::Vector3Stamped& msg)
{
  current_(0) = msg.vector.x;
  current_(1) = msg.vector.y;
  current_(2) = msg.vector.z;
}

/**
 * @brief Pose overwrite callback. It is useful when the dynamics node is used to simulate the DVL to avoid divergence
 * in the robot position over time.
 */
void Dynamics::poseOverwriteCallback(const nav_msgs::Odometry& msg)
{
  p_(0) = msg.pose.pose.position.x;
  p_(1) = msg.pose.pose.position.y;
  p_(2) = msg.pose.pose.position.z;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(p_(3), p_(4), p_(5));
}

/**
 * @brief Timer to check continuity of thrusters and fins setpoints.
 */
void Dynamics::checkActuatorsCallback(const ros::TimerEvent& event)
{
  const double now = event.current_real.toSec();
  if (std::fabs(now - last_thrusters_setpoint_sec_) > 1.0)
  {
    u_ = Eigen::VectorXd::Zero(config_.thrusters_num);
  }
  if (std::fabs(now - last_fins_setpoint_sec_) > 1.0)
  {
    f_ = Eigen::Vector2d::Zero();
  }
}

/**
 * @brief Service callback to reload configuration.
 */
bool Dynamics::reloadConfigServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  getVariableConfig();
  initializeMassMatrix();
  ROS_INFO_STREAM("Params reloaded");
  res.success = true;

  // Publish params after param reload
  std_srvs::Trigger trigger;
  srv_publish_params_.call(trigger);
  if (!trigger.response.success)
  {
    ROS_WARN_STREAM("Publish params did not succeed -> " << trigger.response.message);
  }
  return true;
}

/**
 * @brief Thruster force in robot frame.
 */
Eigen::Vector6d Dynamics::computeThrustersForce(const Eigen::VectorXd& u)
{
  Eigen::VectorXd thruster_forces(config_.thrusters_num);
  for (std::size_t i = 0; i < config_.thrusters_num; ++i)
  {
    if (u(i) >= 0.0)
    {
      thruster_forces(i) =
          bisectionPolySolver(config_.thrusters_poly_positive[i], 0.0, config_.thrusters_max_force_positive[i], u(i));
    }
    else
    {
      thruster_forces(i) =
          -bisectionPolySolver(config_.thrusters_poly_negative[i], 0.0, config_.thrusters_max_force_negative[i], -u(i));
    }
  }
  return config_.thrusters_matrix * thruster_forces;
}

/**
 * @brief Fins force from velocity and fins orientation.
 */
Eigen::Vector6d Dynamics::computeFinsForce(const Eigen::Vector2d& fins, const Eigen::Vector6d& vel)
{
  Eigen::Vector6d f = Eigen::Vector6d::Zero();
  if (!config_.fins_topic.empty())
  {
    // Water velocity on the fins
    double water_vel = vel(0);
    if (water_vel > 0)
    {
      water_vel = std::sqrt(std::pow(vel(0), 2) + (25.0 * vel(0) / (config_.water_density * M_PI * 0.049 * 0.049)));
    }

    // Compute force using new fins model (February of 2015). fins[0] -> left fin, fins[1] -> right fin
    f(0) = -(0.5 * config_.water_density * config_.a_fins * water_vel * std::fabs(water_vel) * config_.k_cd_fins) *
           (std::fabs(std::cos(1 * fins(0))) + std::fabs(std::cos(1 * fins(1))));
    f(1) = 0.0;
    f(2) = +(0.5 * config_.water_density * config_.a_fins * water_vel * std::fabs(water_vel) * config_.k_cl_fins) *
           (std::sin(4.5 * fins(0)) + std::sin(4.5 * fins(1)));
    f(3) = +(0.5 * config_.water_density * config_.a_fins * water_vel * std::fabs(water_vel) * config_.k_cl_fins) *
           (std::sin(4.5 * fins(0)) - std::sin(4.5 * fins(1))) * 0.14;
    f(4) = +(0.5 * config_.water_density * config_.a_fins * water_vel * std::fabs(water_vel) * config_.k_cl_fins) *
           (std::sin(4.5 * fins(0)) + std::sin(4.5 * fins(1))) * 0.65;
    f(5) = 0.0;
  }
  return f;
}

/**
 * @brief Gravity and weight matrix.
 */
Eigen::Vector6d Dynamics::gravityAndBuoyancyForce(const Eigen::Vector6d& pos)
{
  // Weight and buoyancy from [Kg] to [N]
  const double W = STD_GRAVITY * config_.mass;
  const double B = STD_GRAVITY * config_.buoyancy;

  // If the vehicle moves out of the water the buoyancy decreases
  const double corr_pos = pos(2) + config_.radius;  // Corrected z position
  double F = 0.0;
  if (corr_pos >= config_.radius)
  {
    F = B;
  }
  else if (corr_pos > -config_.radius)
  {
    const double r2 = std::pow(config_.radius, 2);
    const double total_area = M_PI * r2;
    const double c = std::sqrt(r2 - std::pow(corr_pos, 2));
    const double area_segment = std::atan2(c, corr_pos) * r2;
    const double area_triangle = corr_pos * c;
    const double area_outside = area_segment - area_triangle;
    F = B * (1.0 - area_outside / total_area);
  }

  // Gravity center position in the robot fixed frame (x',y',z') [m]
  const double cr = std::cos(pos(3));
  const double sr = std::sin(pos(3));
  const double cp = std::cos(pos(4));
  const double sp = std::sin(pos(4));
  const double xb = config_.buoyancy_center(0);
  const double yb = config_.buoyancy_center(1);
  const double zb = config_.buoyancy_center(2);
  Eigen::Vector6d g;
  g << (W - F) * sp, -(W - F) * cp * sr, -(W - F) * cp * cr, F * (yb * cp * cr - zb * cp * sr),
      -F * (zb * sp + xb * cp * cr), F * (xb * cp * sr + yb * sp);
  return g;
}

/**
 * @brief Damping matrix computed from the velocity and damping coefficients.
 */
Eigen::Matrix6d Dynamics::dampingMatrix(const Eigen::Vector6d& vel)
{
  Eigen::Matrix6d damp = Eigen::Matrix6d::Zero();
  for (std::size_t i = 0; i < 6; ++i)
  {
    damp(i, i) = config_.damping(i) + config_.quadratic_damping(i) * std::fabs(vel(i));
  }
  return damp;
}

/**
 * @brief Coriolis matrix computed from the velocity and mass matrix.
 */
Eigen::Matrix6d Dynamics::coriolisMatrix(const Eigen::Vector6d& vel)
{
  const Eigen::Matrix3d s1 = crossMatrix(M_.block<3, 3>(0, 0) * vel.head(3) + M_.block<3, 3>(0, 3) * vel.tail(3));
  const Eigen::Matrix3d s2 = crossMatrix(M_.block<3, 3>(3, 0) * vel.head(3) + M_.block<3, 3>(3, 3) * vel.tail(3));
  Eigen::Matrix6d c = Eigen::Matrix6d::Zero();
  c.block<3, 3>(0, 3) = -s1;
  c.block<3, 3>(3, 0) = -s1;
  c.block<3, 3>(3, 3) = -s2;
  return c;
}

/**
 * @brief Given the velocity and position computes the derivative of the position.
 */
Eigen::Vector6d Dynamics::kinematics(const Eigen::Vector6d& pos, const Eigen::Vector6d& vel)
{
  const double cr = std::cos(pos(3));  // Compute cos, sin and tan only once
  const double sr = std::sin(pos(3));
  double cp = std::cos(pos(4));
  if (std::fabs(cp) < 1e-5)
  {
    cp = 1e-5;  // Avoid division by zero and infinite tangent below
  }
  const double sp = std::sin(pos(4));
  const double tp = sp / cp;
  const double cy = std::cos(pos(5));
  const double sy = std::sin(pos(5));

  Eigen::Matrix3d rec;
  rec << cy * cp, -sy * cr + cy * sp * sr, sy * sr + cy * cr * sp, sy * cp, cy * cr + sr * sp * sy,
      -cy * sr + sp * sy * cr, -sp, cp * sr, cp * cr;

  Eigen::Matrix3d to;
  to << 1.0, sr * tp, cr * tp, 0.0, cr, -sr, 0.0, sr / cp, cr / cp;

  Eigen::Vector6d p_dot;
  p_dot.head(3) = rec * vel.head(3);
  p_dot.tail(3) = to * vel.tail(3);
  return p_dot;
}

/**
 * @brief Given the setpoint for each thruster, the previous velocity and the
 * previous position computes the v_dot.
 */
Eigen::Vector6d Dynamics::inverseDynamic(const Eigen::Vector6d& pos, const Eigen::Vector6d& vel,
                                         const Eigen::VectorXd& u, const Eigen::Vector2d& f)
{
  // Compute current in vehicle frame
  Eigen::Vector6d current = Eigen::Vector6d::Zero();
  const Eigen::Matrix3d rot = cola2::utils::euler2rotation(pos.tail(3));
  current.head(3) = rot.transpose() * current_;

  // Forces from thrusters and fins
  Eigen::Vector6d uf;
  if (config_.use_force_topic)
  {
    uf << force_.wrench.force.x, force_.wrench.force.y, force_.wrench.force.z, force_.wrench.torque.x,
        force_.wrench.torque.y, force_.wrench.torque.z;
  }
  else
  {
    uf = computeThrustersForce(u) + computeFinsForce(f, vel);
  }

  // Gravity force
  const Eigen::Vector6d g = gravityAndBuoyancyForce(pos);

  // Damping and Coriolis forces
  const Eigen::Vector6d cd_v = coriolisMatrix(vel) * vel - dampingMatrix(vel - current) * (vel - current);

  return IM_ * (uf - g - cd_v);  // v_dot
}

/**
 * @brief Main loop operations.
 */
void Dynamics::iterate()
{
  // Simulate tau for thrusters input
  const Eigen::VectorXd u_after_tau =
      (config_.period * u_ + config_.thrusters_tau * old_u_) / (config_.period + config_.thrusters_tau);

  // Runge-Kutta, fixed 4th order
  const Eigen::Vector6d k1_pos = kinematics(p_, v_);
  const Eigen::Vector6d k1_vel = inverseDynamic(p_, v_, old_u_, old_f_);
  const Eigen::Vector6d k2_pos = kinematics(p_ + config_.period * 0.5 * k1_pos, v_ + config_.period * 0.5 * k1_vel);
  const Eigen::Vector6d k2_vel = inverseDynamic(p_ + config_.period * 0.5 * k1_pos, v_ + config_.period * 0.5 * k1_vel,
                                                0.5 * (old_u_ + u_after_tau), 0.5 * (old_f_ + f_));
  const Eigen::Vector6d k3_pos = kinematics(p_ + config_.period * 0.5 * k2_pos, v_ + config_.period * 0.5 * k2_vel);
  const Eigen::Vector6d k3_vel = inverseDynamic(p_ + config_.period * 0.5 * k2_pos, v_ + config_.period * 0.5 * k2_vel,
                                                0.5 * (old_u_ + u_after_tau), 0.5 * (old_f_ + f_));
  const Eigen::Vector6d k4_pos = kinematics(p_ + config_.period * k3_pos, v_ + config_.period * k3_vel);
  const Eigen::Vector6d k4_vel = inverseDynamic(p_ + config_.period * k3_pos, v_ + config_.period * k3_vel,
                                                u_after_tau, f_);

  p_ += config_.period / 6.0 * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);
  v_ += config_.period / 6.0 * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);

  p_(3) = cola2::utils::wrapAngle(p_(3));  // WARN: pitch and roll could go out of the convention here
  p_(4) = cola2::utils::wrapAngle(p_(4));
  p_(5) = cola2::utils::wrapAngle(p_(5));

  old_u_ = u_after_tau;  // This used to be in the thrusters and fins callbacks, which is incorrect
  old_f_ = f_;

  // Publish odometry
  publishOdometry();

  // Publish diagnostics
  thrusters_diag_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  thrusters_diag_.publish();
}

/**
 * @brief Publish odometry, tf and position for Gazebo.
 */
void Dynamics::publishOdometry()
{
  // Header
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = config_.world_frame_id;
  odom.child_frame_id = cola2::ros::getNamespaceNoInitialDash() + "/" + cola2::ros::getUnresolvedNodeName();

  // Position
  odom.pose.pose.position.x = p_(0);
  odom.pose.pose.position.y = p_(1);
  odom.pose.pose.position.z = p_(2);

  // Orientation
  Eigen::Quaterniond quat = cola2::utils::euler2quaternion(p_.tail(3));
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();

  // Velocities
  odom.twist.twist.linear.x = v_(0);
  odom.twist.twist.linear.y = v_(1);
  odom.twist.twist.linear.z = v_(2);
  odom.twist.twist.angular.x = v_(3);
  odom.twist.twist.angular.y = v_(4);
  odom.twist.twist.angular.z = v_(5);

  // Publish
  pub_odom_.publish(odom);

  // Broadcast transform
  geometry_msgs::TransformStamped tfmsg;
  tfmsg.header = odom.header;
  tfmsg.child_frame_id = odom.child_frame_id;
  tfmsg.transform.translation.x = odom.pose.pose.position.x;
  tfmsg.transform.translation.y = odom.pose.pose.position.y;
  tfmsg.transform.translation.z = odom.pose.pose.position.z;
  tfmsg.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_.sendTransform(tfmsg);

  // Publish position for Gazebo
  gazebo_msgs::ModelState gazebo_msg;  // No header in this one
  gazebo_msg.model_name = cola2::ros::getNamespace();
  gazebo_msg.pose = odom.pose.pose;
  gazebo_msg.reference_frame = "world";
  pub_odom_gazebo_.publish(gazebo_msg);
}

/**
 * @brief Get static config from param server. This config is read at the beginning only.
 */
void Dynamics::getStaticConfig()
{
  // World frame
  cola2::ros::getParam("~world_frame_id", config_.world_frame_id, std::string("world_ned"));

  // Period
  cola2::ros::getParam("~period", config_.period, 0.1);
  assert(config_.period > 0.0);
  config_.rate = 1.0 / config_.period;

  // Topics
  cola2::ros::getParam("~fins_topic", config_.fins_topic, std::string(""));

  // Initial pose and velocity
  getParamVector6d("~initial_pose", config_.p0);
  getParamVector6d("~initial_velocity", config_.v0);

  // Number of thrusters
  int thrusters_num;
  cola2::ros::getParam("~number_of_thrusters", thrusters_num, 0);
  assert(thrusters_num >= 0);
  config_.thrusters_num = static_cast<std::size_t>(thrusters_num);
}

/**
 * @brief Get variable config from param server. This config can be changed at runtime.
 */
void Dynamics::getVariableConfig()
{
  // Vehicle properties
  cola2::ros::getParam("~mass", config_.mass, 0.0);
  cola2::ros::getParam("~buoyancy", config_.buoyancy, 0.0);
  cola2::ros::getParam("~radius", config_.radius, 0.0);
  cola2::ros::getParam("~density", config_.water_density, 1030.0);
  getParamMatrix3d("~tensor", config_.tensor);
  getParamVector3d("~buoyancy_center", config_.buoyancy_center);
  getParamVector6d("~damping", config_.damping);
  getParamVector6d("~quadratic_damping", config_.quadratic_damping);

  // Thrusters
  getParamMatrixXd("~thrusters_matrix", config_.thrusters_matrix, 6);
  cola2::ros::getParam("~thrusters_tau", config_.thrusters_tau, 0.0);
  config_.thrusters_limiter.clear();
  config_.thrusters_limiter.resize(config_.thrusters_num, 1.0);
  cola2::ros::getParamVector("~thrusters_limiter", config_.thrusters_limiter);
  cola2::ros::getParam("~thrusters_max_step", config_.thrusters_max_step, 1.0);
  cola2::ros::getParam("~thrusters_symmetric", config_.thrusters_symmetric, true);
  config_.thrusters_max_force_positive.clear();
  config_.thrusters_max_force_negative.clear();
  config_.thrusters_poly_positive.clear();
  config_.thrusters_poly_negative.clear();
  for (std::size_t i = 0; i < config_.thrusters_num; ++i)
  {
    // Max forces
    double max_force_positive, max_force_negative;
    cola2::ros::getParam(std::string("~thruster_") + std::to_string(i + 1) + std::string("_max_force_positive"),
                         max_force_positive, 0.0);
    cola2::ros::getParam(std::string("~thruster_") + std::to_string(i + 1) + std::string("_max_force_negative"),
                         max_force_negative, 0.0);
    config_.thrusters_max_force_positive.push_back(max_force_positive);
    config_.thrusters_max_force_negative.push_back(max_force_negative);

    // Force to setpoint polys
    std::vector<double> thruster_poly_positive, thruster_poly_negative;
    cola2::ros::getParamVector(std::string("~thruster_") + std::to_string(i + 1) + std::string("_poly_positive"),
                               thruster_poly_positive);
    cola2::ros::getParamVector(std::string("~thruster_") + std::to_string(i + 1) + std::string("_poly_negative"),
                               thruster_poly_negative);

    std::map<std::string, double> params_positive;
    params_positive["n_dof"] = thruster_poly_positive.size();
    for (std::size_t j = 0; j < thruster_poly_positive.size(); ++j)
    {
      params_positive[std::to_string(j)] = thruster_poly_positive[j];
    }
    config_.thrusters_poly_positive.push_back(Poly("dynamics_positive_poly"));
    config_.thrusters_poly_positive[i].setParameters(params_positive);

    std::map<std::string, double> params_negative;
    params_negative["n_dof"] = thruster_poly_negative.size();
    for (std::size_t j = 0; j < thruster_poly_negative.size(); ++j)
    {
      params_negative[std::to_string(j)] = thruster_poly_negative[j];
    }
    config_.thrusters_poly_negative.push_back(Poly("dynamics_negative_poly"));
    config_.thrusters_poly_negative[i].setParameters(params_negative);
  }

  // Fins
  if (!config_.fins_topic.empty())
  {
    cola2::ros::getParam("~a_fins", config_.a_fins, 0.0);
    cola2::ros::getParam("~k_cd_fins", config_.k_cd_fins, 0.0);
    cola2::ros::getParam("~k_cl_fins", config_.k_cl_fins, 0.0);
    cola2::ros::getParam("~max_fins_angle", config_.max_fins_angle, 0.0);
  }

  // Force
  cola2::ros::getParam("~use_force_topic", config_.use_force_topic, false);
}

/**
 * @brief Returns the rate. Used in the main while loop that calls iterate().
 */
double Dynamics::getRate() const
{
  return config_.rate;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dynamics");
  Dynamics node;
  ros::Rate rate(node.getRate());
  while (ros::ok())
  {
    node.iterate();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
