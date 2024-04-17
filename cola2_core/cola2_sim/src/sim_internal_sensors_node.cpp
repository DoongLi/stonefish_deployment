/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/Bool.h>

// TODO: This node should be generic. It means that it should read from a yaml file how many temperature, water ...
//       ... and humidity sensors have to simulate (i.e., generate diagnostics) and with which names.

class SimInternalSensors
{
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_bat_status_;
  ros::Publisher pub_bat_humidity_, pub_bat_pressure_, pub_bat_temperature_, pub_bat_water_detected_;
  ros::Publisher pub_pc_humidity_, pub_pc_pressure_, pub_pc_temperature_, pub_pc_water_detected_;
  ros::Timer timer_pub_pc_, timer_pub_bat_, timer_pub_bat_s_;
  cola2::ros::DiagnosticHelper diagnostic_bat_, diagnostic_bat_cylinder_, diagnostic_pc_cylinder_;
  double battery_level_;
  double battery_consumption_per_second_;
  const double update_battery_time_;

  // Methods
  void pubPCCallback(const ros::TimerEvent&);
  void pubBatCallback(const ros::TimerEvent&);
  void pubBatSCallback(const ros::TimerEvent&);

 public:
  SimInternalSensors();
};

SimInternalSensors::SimInternalSensors()
  : nh_("~")
  , diagnostic_bat_(nh_, "batteries", cola2::ros::getUnresolvedNodeName())
  , diagnostic_bat_cylinder_(nh_, "battery_control_board", cola2::ros::getUnresolvedNodeName())
  , diagnostic_pc_cylinder_(nh_, "main_control_board", cola2::ros::getUnresolvedNodeName())
  , battery_level_(93.0)
  , battery_consumption_per_second_(0.004)
  , update_battery_time_(1.0)
{
  // Get config
  cola2::ros::getParam("~initial_battery_level", battery_level_);
  cola2::ros::getParam("~battery_consumption_per_second", battery_consumption_per_second_);

  // Diagnostics
  diagnostic_bat_.setEnabled(true);
  diagnostic_bat_cylinder_.setEnabled(true);
  diagnostic_pc_cylinder_.setEnabled(true);

  // Publishers
  const std::string ns = cola2::ros::getNamespace();
  pub_bat_status_ = nh_.advertise<sensor_msgs::BatteryState>(ns + "/batteries/status", 1);
  pub_bat_humidity_ = nh_.advertise<sensor_msgs::RelativeHumidity>(ns + "/batteries_cylinder/humidity", 1);
  pub_bat_pressure_ = nh_.advertise<sensor_msgs::FluidPressure>(ns + "/batteries_cylinder/pressure", 1);
  pub_bat_temperature_ = nh_.advertise<sensor_msgs::Temperature>(ns + "/batteries_cylinder/temperature", 1);
  pub_bat_water_detected_ = nh_.advertise<std_msgs::Bool>(ns + "/batteries_cylinder/water_detected", 1);
  pub_pc_humidity_ = nh_.advertise<sensor_msgs::RelativeHumidity>(ns + "/pc_cylinder/humidity", 1);
  pub_pc_pressure_ = nh_.advertise<sensor_msgs::FluidPressure>(ns + "/pc_cylinder/pressure", 1);
  pub_pc_temperature_ = nh_.advertise<sensor_msgs::Temperature>(ns + "/pc_cylinder/temperature", 1);
  pub_pc_water_detected_ = nh_.advertise<std_msgs::Bool>(ns + "/pc_cylinder/water_detected", 1);

  // Timers
  timer_pub_pc_ = nh_.createTimer(ros::Duration(1.0), &SimInternalSensors::pubPCCallback, this);
  timer_pub_bat_ = nh_.createTimer(ros::Duration(1.0), &SimInternalSensors::pubBatCallback, this);
  timer_pub_bat_s_ = nh_.createTimer(ros::Duration(update_battery_time_), &SimInternalSensors::pubBatSCallback, this);

  ROS_INFO("Initialized");
}

void
SimInternalSensors::pubPCCallback(const ros::TimerEvent& event)
{
  // PC Cylinder humidity
  sensor_msgs::RelativeHumidity hum_msg;
  hum_msg.header.stamp = event.current_real;
  hum_msg.relative_humidity = 45.0;
  hum_msg.variance = 0.0;
  pub_pc_humidity_.publish(hum_msg);

  // PC Cylinder temperature
  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = event.current_real;
  temp_msg.temperature = 57.4;
  temp_msg.variance = 0.0;
  pub_pc_temperature_.publish(temp_msg);

  // PC Cylinder pressure
  sensor_msgs::FluidPressure press_msg;
  press_msg.header.stamp = event.current_real;
  press_msg.fluid_pressure = 101300.0;
  press_msg.variance = 0.0;
  pub_pc_pressure_.publish(press_msg);

  // PC Cylinder Water detected
  std_msgs::Bool water_detected_msg;
  water_detected_msg.data = false;
  pub_pc_water_detected_.publish(water_detected_msg);

  // Diagnostics
  diagnostic_pc_cylinder_.addKeyValue("temperature", temp_msg.temperature);
  diagnostic_pc_cylinder_.addKeyValue("pressure", press_msg.fluid_pressure);
  diagnostic_pc_cylinder_.addKeyValue("water_internal_inside", false);
  diagnostic_pc_cylinder_.addKeyValue("water_external_inside", false);
  diagnostic_pc_cylinder_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  diagnostic_pc_cylinder_.reportValidData(event.current_real);
  diagnostic_pc_cylinder_.publish(event.current_real);
}

void
SimInternalSensors::pubBatCallback(const ros::TimerEvent& event)
{
  // Batteries Cylinder humidity
  sensor_msgs::RelativeHumidity hum_msg;
  hum_msg.header.stamp = event.current_real;
  hum_msg.relative_humidity = 38.0;
  hum_msg.variance = 0.0;
  pub_bat_humidity_.publish(hum_msg);

  // Batteries Cylinder temperature
  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = event.current_real;
  temp_msg.temperature = 31.2;
  temp_msg.variance = 0.0;
  pub_bat_temperature_.publish(temp_msg);

  // Batteries Cylinder pressure
  sensor_msgs::FluidPressure press_msg;
  press_msg.header.stamp = event.current_real;
  press_msg.fluid_pressure = 101325.0;
  press_msg.variance = 0.0;
  pub_bat_pressure_.publish(press_msg);

  // Batteries Cylinder Water detected
  std_msgs::Bool water_detected_msg;
  water_detected_msg.data = false;
  pub_bat_water_detected_.publish(water_detected_msg);

  // Diagnostics
  diagnostic_bat_cylinder_.addKeyValue("status_indicator", 0);
  diagnostic_bat_cylinder_.addKeyValue("temperature", temp_msg.temperature);
  diagnostic_bat_cylinder_.addKeyValue("pressure", press_msg.fluid_pressure);
  diagnostic_bat_cylinder_.addKeyValue("water_internal_inside", false);
  diagnostic_bat_cylinder_.addKeyValue("water_external_inside", false);
  diagnostic_bat_cylinder_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  diagnostic_bat_cylinder_.reportValidData(event.current_real);
  diagnostic_bat_cylinder_.publish(event.current_real);
}

void
SimInternalSensors::pubBatSCallback(const ros::TimerEvent& event)
{
  sensor_msgs::BatteryState battery_msg;
  battery_msg.header.stamp = event.current_real;
  battery_msg.voltage = 30.6;
  battery_msg.current = -30.02;
  battery_msg.charge = battery_level_;
  battery_msg.capacity = 0.0;
  battery_msg.design_capacity = 0.0;
  battery_msg.percentage = 0.0;
  battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_msg.present = true;
  pub_bat_status_.publish(battery_msg);

  // Reduce battery level
  battery_level_ -= battery_consumption_per_second_ * update_battery_time_;
  if (battery_level_ < 0.0)
    battery_level_ = 0.0;

  // Publish diagnostic message
  diagnostic_bat_.addKeyValue("charge", battery_msg.charge);
  diagnostic_bat_.addKeyValue("temperature", 30.0);
  diagnostic_bat_.addKeyValue("voltage", battery_msg.voltage);
  diagnostic_bat_.addKeyValue("minutes", battery_level_ * 3.15);
  diagnostic_bat_.addKeyValue("status", "DISCHARGING");
  if (battery_msg.charge > 15.0)
      diagnostic_bat_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::OK);
  else if (battery_msg.charge > 5.0)
      diagnostic_bat_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::WARN, "Low Battery");
  else
      diagnostic_bat_.setLevelAndMessage(diagnostic_msgs::DiagnosticStatus::ERROR, "Recharge battery!");
  diagnostic_bat_.reportValidData(event.current_real);
  diagnostic_bat_.publish(event.current_real);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sim_internal_sensors");
  SimInternalSensors sim_internal_sensors;
  ros::spin();
  return 0;
}
