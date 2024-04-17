/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_NAV_EKF_BASE_ROS_H_
#define COLA2_NAV_EKF_BASE_ROS_H_

// messages subscribed
#include <cola2_msgs/BodyForceReq.h>                  // force velocity model
#include <cola2_msgs/DVL.h>                           // dvl
#include <cola2_msgs/Float32Stamped.h>                // force velocity model
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // usbl
#include <sensor_msgs/FluidPressure.h>                // depth
#include <sensor_msgs/Imu.h>                          // imu
#include <sensor_msgs/NavSatFix.h>                    // gps
#include <sensor_msgs/Range.h>                        // altitude
#include <std_msgs/Float32.h>                         // sound velocity
// messages published
#include <cola2_msgs/NavSts.h>                 // custom navigation
#include <diagnostic_msgs/DiagnosticStatus.h>  // diagnostics
#include <geometry_msgs/PoseStamped.h>         // gps_ned, usbl_ned
#include <nav_msgs/Odometry.h>                 // odometry
// services
#include <std_srvs/Trigger.h>
// all
#include <cola2_lib/utils/ned.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_lib_ros/transform_handler.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>

#include "./ekf_base.h"
#include "./transformations.h"

void showInfoOrWarning(const std::string& text, const bool use_info);

namespace
{
constexpr double TIME_BETWEEN_PUBLISHING = 1.0 / 20.0;  //!< minimum time between publishNavigation()
constexpr double USBL_KEEP_TIME = 10.0;                 //!< seconds to keep position history for delayed USBLs
constexpr size_t ALTITUDE_WINDOW_SIZE = 4;              //!< altitude window to check for valid measurements
constexpr double INIT_TIME_NO_WARNINGS = 20.0;  //!< seconds before warnings are displayed (info messages before)
}  // namespace

/**
 * \brief EKF including all the ROS functionalities.
 * It loads parameters, publishes state
 */
class EKFBaseROS : public EKFBase
{
protected:
  // ROS transforms and frames
  cola2::ros::TransformHandler tf_handler_;  //!< provide tfs
  tf::TransformBroadcaster tf_broadcast_;    //!< to publish world->vehicle
  std::string ns_;                           //!< namespace we are in
  std::string frame_world_ = "world_ned";    //!< frame where AUV is located
  std::string frame_vehicle_;                //!< frame of the vehicle (ns/base_link)

  // ROS Publishers
  ros::Publisher pub_odom_;             //!< odometry message
  ros::Publisher pub_nav_;              //!< custom navigation message
  ros::Publisher pub_gps_ned_;          //!< GPS projected in current NED
  ros::Publisher pub_usbl_ned_;         //!< USBL projected in current NED
  double last_publication_time_ = 0.0;  //!< Last time navigation was published

  // ROS Services
  ros::ServiceServer srv_reload_ned_;               //!< reload ned config and reuse current filter
  ros::ServiceServer srv_reload_params_;            //!< same as reset navigation
  ros::ServiceServer srv_reset_navigation_;         //!< realoads params and sets the filter to initial state
  ros::ServiceServer srv_set_depth_sensor_offset_;  //!< compute the depth sensor offset
  ros::ServiceClient srv_publish_params_;           //!< service to tell param_logger that parameters have been updated

  // Status
  double pressure_meters_ = 0.0;    //!< last measured pressure
  double altitude_ = 0.0;           //!< altitude of the vehicle above floor
  double sound_velocity_ = 1500.0;  //!< sound velocity

  // Other
  ros::Timer timer_;       //!< loop to keep checking diagnostics
  cola2::utils::NED ned_;  //!< reference frame where the AUV is
  double start_time_;      //!< start time of the filter

  struct StampedPosition
  {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    StampedPosition(const double& t, const Eigen::Vector3d& pos, const Eigen::Vector3d& rpy)
    {
      time = t;
      position = pos;
      orientation = rpy;
    }
  };
  std::vector<StampedPosition> last_usbl_positions_;  //!< delayed USBL

  // Debug in output
  std::ofstream ofh_;  //!< output file for debug purposes

  /**
   * \brief Function called by the check diagnostics timer.
   *
   * \param e Event triggered by a ROS timer
   */
  void checkDiagnostics(const ros::TimerEvent& e);

public:
  /**
   * \brief Reset filter to initial status
   */
  void resetFilter();

protected:
  /**
   * \brief Get position increment for a position in the past.
   *
   * \param time Timestamp where the initial position is (current position is at last_prediction_)
   * \return Time increment, position increment in x, position increment in y
   */
  Eigen::Vector3d getPositionIncrementFrom(const double time) const;

protected:
  ros::NodeHandle nh_ = ros::NodeHandle("~");  //!< ROS node handler
  bool online_;                                //!< navigator running online

  // Init flags
  bool init_depth_offset_ = false;  //!< init depth offset
  bool init_gps_ = false;           //!< init sensor gps
  bool init_depth_ = false;         //!< init sensor depth
  bool init_dvl_ = false;           //!< init sensor dvl
  bool init_imu_ = false;           //!< init sensor imu
  bool init_ned_ = false;           //!< init NED
  bool ned_error_ = false;          //!< not enough good smaples to init NED from GPS

  // Diagnostics
  cola2::ros::DiagnosticHelper diag_help_;    //!< ease publishing diagnostics
  double last_gps_time_ = 0.0;                //!< last time gps received
  double last_usbl_time_ = 0.0;               //!< last time usbl received
  double last_depth_time_ = 0.0;              //!< last time depth received
  double last_dvl_time_ = 0.0;                //!< last time dvl received
  double last_imu_time_ = 0.0;                //!< last time imu received
  double last_altitude_time_ = 0.0;           //!< last time altitude received
  double last_ekf_init_time_ = 0.0;           //!< last time ekf initialized
  size_t gps_samples_ = 0;                    //!< number of valid GPS samples gathered
  size_t gps_samples_wrong_before_init_ = 0;  //!< number of invalid GPS samples before GPS init
  double last_gps_sample_time_ = 0.0;         //!< last time of GPS valid sample

  // Configs from param server
public:
  /**
   * \brief Load config from ROS param server.
   *
   * \param show Print on screen the loaded parameters
   */
  std_srvs::TriggerResponse getConfig(const bool show = false);

protected:
  /**
   * \brief Struct with all the configuration that need to be loaded
   */
  struct Config
  {
    // Flags
    bool initialize_filter_from_gps_;  //!< initial filter position taken from GPS
    int gps_samples_to_init_;          //!< number of valid GPS messages before initializing GPS
    bool use_gps_data_;                //!< use GPS mesurements in the filter
    bool use_usbl_data_;               //!< use USBL measurements in the filter
    bool use_depth_data_;              //!< use depth measurements in the filter
    bool use_dvl_data_;                //!< use DVL measurements in the filter
    bool enable_debug_;                //!< record debug information into and external file in $HOME dir
    // NED
    double ned_latitude_;   //!< manually fixed NED origin latitude
    double ned_longitude_;  //!< manually fixed NED origin longitude
    // Depth offset
    bool initialize_depth_sensor_offset_;   //!< compute depth sensor offset when vehicle is at surface
    double surface2depth_sensor_distance_;  //!< distance from depth sensor to water surface
    double depth_sensor_offset_;            //!< manually/computed set depth sensor offset value
    // Sensors
    double dvl_max_v_;      //!< maximum velocity of accepted DVL measurements
    double water_density_;  //!< density of the water (used in pressure sensor computations)
    // DVL fallback
    double dvl_fallback_delay_;  //!< minimum time to receive invalid DVL measurments to use the dvl_fallback callbacks
    // Covariances
    std::vector<double> initial_state_covariance_;     //!< initial covariance of the filter
    std::vector<double> prediction_model_covariance_;  //!< prediction covariance (Q in the prediction equations)
    // Diagnostics
    double min_diagnostics_frequency_;  //!< minimum frequency required by diagnostic messages
  };
  Config config_;  //!< config loaded by getConfig()

public:
  // *****************************************
  // Constructor and destructor
  // *****************************************
  /**
   * \brief Constructor.
   *
   * \param initial_state_vector_size Size in rows of the state vector
   * \param online Run the filter online (enable publishers, subscribers and services that require init_node)
   */
  explicit EKFBaseROS(const unsigned int initial_state_vector_size, const bool online = true);
  /**
   * \brief Destructor.
   */
  virtual ~EKFBaseROS() noexcept;

  // *****************************************
  // Updates from ROS messages
  // *****************************************
  // Messages that affect the filter
  /**
   * \brief Callback to update the filter with a GPS message.
   *
   * Also publishes any valid gps measurement at the water surface:
   * - Arrow up when the gps has not updated the filter.
   * - Arrow with vehicle yaw when the gps has updated the filter.
   *
   * \param msg Recevied message
   */
  void updatePositionGPSMsg(const sensor_msgs::NavSatFix& msg);
  bool updatePositionGPSMsgImpl(const sensor_msgs::NavSatFix& msg);

  /**
   * \brief USBL update status.
   */
  enum class USBLStatus
  {
    OutOfBuffer,      //!< Cannot find the stamp in the last_usbl_positions_ buffer.
    NotUpdated,       //!< Stamp found in buffer but filter did not update.
    Updated,          //!< Filter updated with USBL measurement.
    IQUAviewInvalid,  //!< IQUAview sends invalid timestamp.
  };
  /**
   * \brief Callback to update the filter with a USBL message.
   *
   * Also publishes the USBL measure at the vehicle depth:
   * - Arrow down when the usbl is outside the past positions buffer (raw measure).
   * - Arrow up when the usbl is in the buffer and has not updated the filter (translated measure).
   * - Arrow with vehicle yaw when the usbl has updated the filter (translated measure).
   *
   * \param msg Recevied message
   */
  void updatePositionUSBLMsg(const geometry_msgs::PoseWithCovarianceStamped& msg);
  bool updatePositionUSBLMsgImpl(const geometry_msgs::PoseWithCovarianceStamped& msg);
  /**
   * \brief Callback to update the filter with a depth message
   *
   * \param msg Recevied message
   */
  void updatePositionDepthMsg(const sensor_msgs::FluidPressure& msg);
  /**
   * \brief Callback to update the filter with a DVL message
   *
   * \param msg Recevied message
   */
  void updateVelocityDVLMsg(const cola2_msgs::DVL& msg);
  /**
   * \brief Callback to update the filter with a fallback DVL message
   *
   * This callback only works when invalid DVL messages have been received at leat for config_.dvl_fallback_delay_
   *
   * \param msg Recevied message
   */
  void updateVelocityDVLFallbackMsg(const cola2_msgs::DVL& msg);
  /**
   * \brief Implementation of the DVL message update depending on the source of the message
   *
   * \param msg Recevied message
   * \param from_dvl The message comes from a real DVL sensor
   */
  void updateVelocityDVLMsgImpl(const cola2_msgs::DVL& msg, const bool from_dvl);
  /**
   * \brief Callback to update the filter with an IMU message
   *
   * \param msg Recevied message
   */
  void updateIMUMsg(const sensor_msgs::Imu& msg);
  // Others
  /**
   * \brief Callback to update the filter with a sound velocity message
   *
   * \param msg Recevied message
   */
  void updateSoundVelocityMsg(const cola2_msgs::Float32Stamped& msg);
  /**
   * \brief Callback to update the filter with an altitude message
   *
   * \param msg Recevied message
   */
  void updateAltitudeMsg(const sensor_msgs::Range& msg);

  // *****************************************
  // Publishers
  // *****************************************
  /**
   * \brief Publish the filter state
   *
   * This publications includes:
   *   - Odometry message
   *   - Navigation message
   *   - Transform from world frame to vehicle base_link frame
   *
   * \param stamp Timestamp of each published message
   */
  void publishNavigation(const ros::Time& stamp);
  /**
   * \brief Publish a GPS measurement in the current NED frame.
   *
   * Orientation of the StampedPose:
   * - not valid: Arrow up.
   * - valid: Arrow with vehicle yaw.
   *
   * \param stamp Timestamp of the published message
   * \param ned The measurement in a NED frame
   * \param valid The measurement has updated the filter (affects orientation output).
   */
  void publishGPSNED(const ros::Time& stamp, const Eigen::Vector3d& ned, const bool& valid) const;
  /**
   * \brief Publish a USBL measurement in the current NED frame.
   *
   * Orientation of the StampedPose:
   * - USBLStatus::OutOfBuffer: Arrow down.
   * - USBLStatus::NotUpdated: Arrow up.
   * - USBLStatus::Updated: Arrow with vehicle yaw.
   *
   * \param stamp Timestamp of the published message
   * \param ned The measurement in a NED frame
   * \param status Status of the measurement against the filter (affects orientation output).
   */
  void publishUSBLNED(const ros::Time& stamp, const Eigen::Vector3d& ned, const USBLStatus& status) const;

  // *****************************************
  // Services
  // *****************************************
  /**
   * \brief Reload only NED information from the param server.
   *
   * \param req Request to the service
   * \param res Response to the service
   * \return Correct processing of the service
   */
  bool srvReloadNED(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  /**
   * \brief Reset navigation to the initial state and reload the configuration
   *
   * \param req Request to the service
   * \param res Response to the service
   * \return Correct processing of the service
   */
  bool srvResetNavigation(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  /**
   * \brief Compute the depth sensor offset in the current conditions
   *
   * \param req Request to the service
   * \param res Response to the service
   * \return Correct processing of the service
   */
  bool srvSetDepthSensorOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // *****************************************
  // Offline
  // *****************************************
  /**
   * \brief Load transforms data from file. Useful when not working online in ROS since we cannot query /tf topic.
   *
   * The format of each trasnform is a line in the file with the follwoing parts:
   *   parent_frame child_frame tx ty tz qx qy qz qw
   *
   * \param fname Filename containing the transforms to load
   */
  void loadTranformsFromFile(const std::string& fname);

  /**
   * \brief Use a xy position as GPS init (useful to set bagfile odometry to init filter underwater).
   *
   * \param xy Position to set in the filter as GPS init.
   */
  void setPositionXYasGPSinit(const Eigen::Vector2d& xy);

  // *****************************************
  // To be implemented
  // *****************************************
protected:
  // From EKFBase
  virtual void setPositionXY(const Eigen::Vector2d& xy) = 0;
  virtual void computePredictionMatrices(const double dt) = 0;
  virtual void normalizeState() = 0;
  virtual bool updatePositionXY(const double t, const Eigen::Vector2d& pose_xy, const Eigen::Matrix2d& cov) = 0;
  virtual bool updatePositionZ(const double t, const Eigen::Vector1d& pose_z, const Eigen::Matrix1d& cov) = 0;
  virtual bool updateOrientation(const double t, const Eigen::Vector3d& rpy, const Eigen::Matrix3d& cov) = 0;
  virtual bool updateVelocity(const double t, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov,
                              const bool from_dvl = true) = 0;
  virtual bool updateOrientationRate(const double t, const Eigen::Vector3d& rate, const Eigen::Matrix3d& cov) = 0;

public:
  double getAltitude() const;
  // From EKFBase
  virtual Eigen::Vector3d getPosition() const = 0;
  virtual Eigen::Vector3d getVelocity() const = 0;
  virtual Eigen::Vector3d getEuler() const = 0;
  virtual Eigen::Vector3d getAngularVelocity() const = 0;
  virtual Eigen::Matrix3d getPositionUncertainty() const = 0;
  virtual Eigen::Matrix3d getVelocityUncertainty() const = 0;
  virtual Eigen::Matrix3d getOrientationUncertainty() const = 0;
  virtual Eigen::Matrix3d getAngularVelocityUncertainty() const = 0;
};

#endif  // COLA2_NAV_EKF_BASE_ROS_H_
