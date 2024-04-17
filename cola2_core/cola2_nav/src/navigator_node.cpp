/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_nav/ekf_position_velocity.h>
#include <ros/ros.h>

class NavigatorNode : public EKFPositionVelocity
{
private:
  // Subscribers
  ros::Subscriber sub_gps_;             // [x y]
  ros::Subscriber sub_usbl_;            // [x y]
  ros::Subscriber sub_pressure_;        // [z]
  ros::Subscriber sub_dvl_;             // [vx vy vz]
  ros::Subscriber sub_dvl_fallback_;    // [vx vy vz]
  ros::Subscriber sub_imu_;             // [roll pitch yaw vroll vpitch vyaw]
  ros::Subscriber sub_sound_velocity_;  // sound velocity from SVS
  ros::Subscriber sub_altitude_;        // altitude from seafloor

public:
  /**
   * \brief Constructor that relates all sensors to their callbacks.
   */
  NavigatorNode();
  /**
   * \brief Destructor.
   */
  ~NavigatorNode();
};

NavigatorNode::NavigatorNode()
{
  // clang-format off
  // Init subscribers
  sub_gps_ = nh_.subscribe("gps", 2, &EKFBaseROS::updatePositionGPSMsg, reinterpret_cast<EKFBaseROS*>(this));
  sub_usbl_ = nh_.subscribe("usbl", 2, &EKFBaseROS::updatePositionUSBLMsg,  reinterpret_cast<EKFBaseROS*>(this));
  sub_pressure_ = nh_.subscribe("pressure", 2, &EKFBaseROS::updatePositionDepthMsg,  reinterpret_cast<EKFBaseROS*>(this));
  sub_dvl_ = nh_.subscribe("dvl", 2, &EKFBaseROS::updateVelocityDVLMsg,  reinterpret_cast<EKFBaseROS*>(this));
  sub_imu_ = nh_.subscribe("imu", 2, &EKFBaseROS::updateIMUMsg,  reinterpret_cast<EKFBaseROS*>(this));
  if (config_.dvl_fallback_delay_ > 0.0)
  {
    ROS_INFO("Subscribing to 'dvl_fallback'");
    sub_dvl_fallback_ = nh_.subscribe("dvl_fallback", 2, &EKFBaseROS::updateVelocityDVLFallbackMsg,  reinterpret_cast<EKFBaseROS*>(this));
  }
  // Other data
  sub_sound_velocity_ = nh_.subscribe("sound_velocity", 2, &EKFBaseROS::updateSoundVelocityMsg,  reinterpret_cast<EKFBaseROS*>(this));
  sub_altitude_ = nh_.subscribe("altitude", 2, &EKFBaseROS::updateAltitudeMsg,  reinterpret_cast<EKFBaseROS*>(this));
  // clang-format on

  ROS_INFO("Initialized");
}

NavigatorNode::~NavigatorNode()
{
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "navigator");
  NavigatorNode node;
  ros::spin();  // spin until architecture stops
  return 0;
}
