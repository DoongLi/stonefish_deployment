/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_safety/safety_rules/navigator.h>

#include <algorithm>

namespace SafetyRules
{
Navigator::Navigator(const std::string& rule_name, ros::NodeHandle* nh_ptr)
  : SafetyRuleBaseClass(rule_name)
  , last_altitude_wwr_(0.0)
  , filter_init_(false)
  , last_nav_data_(ros::Time::now().toSec())
  , last_imu_data_(0.0)
  , last_depth_data_(0.0)
  , last_altitude_data_(0.0)
  , last_dvl_data_(0.0)
  , last_gps_data_(0.0)
  , imu_data_timeout_(0.0)
  , depth_data_timeout_(0.0)
  , altitude_data_timeout_(0.0)
  , dvl_data_timeout_(0.0)
  , gps_data_timeout_(0.0)
  , min_frequency_(0.0)
  , last_surfaced_(0.0)
  , first_no_altitude_(0.0)
{
  loadConfigFromParamServer();

  const ParseList parse_list(
      { { std::string("/navigation/") + rule_name_, "filter_init", "filter_init", DataType::Bool },
        { std::string("/navigation/") + rule_name_, "last_imu_data", "last_imu_data", DataType::Double },
        { std::string("/navigation/") + rule_name_, "last_depth_data", "last_depth_data", DataType::Double },
        { std::string("/navigation/") + rule_name_, "last_altitude_data", "last_altitude_data", DataType::Double },
        { std::string("/navigation/") + rule_name_, "last_dvl_data", "last_dvl_data", DataType::Double },
        { std::string("/navigation/") + rule_name_, "last_gps_data", "last_gps_data", DataType::Double },
        { std::string("/navigation/") + rule_name_, "frequency", "frequency", DataType::Double } });
  setParseList(parse_list);

  // Subscriber
  sub_nav_ = nh_ptr->subscribe(cola2::ros::getNamespace() + "/navigator/navigation", 10, &Navigator::navCallback, this);
  sub_wwr_ = nh_ptr->subscribe(cola2::ros::getNamespace() + "/controller/world_waypoint_req", 10,
                               &Navigator::wwrCallback, this);
}

void Navigator::parseDiagnostics()
{
  const bool valid_diagnostics = hasBool("filter_init") && hasDouble("last_imu_data") && hasDouble("last_depth_data") &&
                                 hasDouble("last_altitude_data") && hasDouble("last_dvl_data") &&
                                 hasDouble("last_gps_data");
  if (valid_diagnostics)
  {
    last_valid_diagnostics_data_ = last_diagnostic_;
    filter_init_ = getBool("filter_init");
    last_imu_data_ = getDouble("last_imu_data");
    last_depth_data_ = getDouble("last_depth_data");
    last_altitude_data_ = getDouble("last_altitude_data");
    last_dvl_data_ = getDouble("last_dvl_data");
    last_gps_data_ = getDouble("last_gps_data");
  }
}

void Navigator::navCallback(const cola2_msgs::NavSts& nav)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(nav))
  {
    return;
  }

  last_nav_data_ = nav.header.stamp.toSec();
  if ((last_surfaced_ == 0.0) && (nav.position.depth <= SURFACE_DEPTH))
    last_surfaced_ = last_nav_data_;
  else if (nav.position.depth > SURFACE_DEPTH)
    last_surfaced_ = 0.0;
}

void Navigator::wwrCallback(const cola2_msgs::WorldWaypointReq& wwr)
{
  if (wwr.altitude_mode)
    last_altitude_wwr_ = wwr.header.stamp.toSec();
}

void Navigator::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
{
  // Clear level, message and status code bit
  level_ = SafetyLevel::NONE;
  message_.clear();

  // Keep track of the first stamp where altitude is missing
  if ((last_altitude_data_ > altitude_data_timeout_) && (stamp.toSec() - last_altitude_wwr_ < 2.0))
  {
    if (first_no_altitude_ == 0.0)
      first_no_altitude_ = stamp.toSec();
  }
  else
    first_no_altitude_ = 0.0;

  // Check data
  if (last_valid_config_ == 0.0)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("invalid config");
  }
  else if (last_valid_diagnostics_data_ == 0.0)
  {
    message_ = createMessage("waiting for valid diagnostics");
    if (stamp.toSec() - last_valid_config_ < INIT_TIME)
      level_ = SafetyLevel::INFORMATIVE;
    else
      level_ = SafetyLevel::ABORT_AND_SURFACE;
  }
  else
  {
    // Create a list of messages to report
    std::vector<std::string> messages;

    // Missing diagnostics
    if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_ESCALATED_TIME)
    {
      level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::EMERGENCY_SURFACE));
      messages.push_back(createMessage("too much time without valid diagnostics, escalated to emergency"));
    }
    else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_TIME)
    {
      level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyRules::SafetyLevel::ABORT_AND_SURFACE));
      messages.push_back(createMessage("too much time without valid diagnostics"));
    }

    // Filter not initialized
    if (!filter_init_)
    {
      messages.push_back(createMessage("filter not initialized"));
      level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
    }
    else
    {
      // Navigation data timeout
      if (stamp.toSec() - last_nav_data_ > nav_data_timeout_)
      {
        messages.push_back(createMessage("navigation data age above threshold"));
        if (stamp.toSec() - last_valid_config_ < INIT_TIME)
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
        else
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::EMERGENCY_SURFACE));
      }

      // Depth data timeout
      if (last_depth_data_ > depth_data_timeout_)
      {
        messages.push_back(createMessage("depth data age above threshold"));
        if (stamp.toSec() - last_valid_config_ < INIT_TIME)
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
        else
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::EMERGENCY_SURFACE));
      }

      // IMU data timeout
      if (last_imu_data_ > imu_data_timeout_)
      {
        messages.push_back(createMessage("IMU data age above threshold"));
        if (stamp.toSec() - last_valid_config_ < INIT_TIME)
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
        else
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::EMERGENCY_SURFACE));
      }

      // Altitude data timeout
      if ((first_no_altitude_ != 0.0) && (stamp.toSec() - first_no_altitude_ > 3.0))
      {
        level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::ABORT_AND_SURFACE));
        messages.push_back(createMessage("altitude data age above threshold"));
        StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::NO_ALTITUDE_ERROR, true);
      }

      // DVL data timeout
      if (last_dvl_data_ > dvl_data_timeout_)
      {
        messages.push_back(createMessage("DVL data age above threshold"));
        if (stamp.toSec() - last_valid_config_ < INIT_TIME)
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
        else
          level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::ABORT_AND_SURFACE));
      }

      // GPS data timeout
      if ((last_surfaced_ != 0.0) && (stamp.toSec() - last_surfaced_ > GPS_SURFACE_TIME) &&
          (last_gps_data_ > gps_data_timeout_))
      {
        level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::ABORT_AND_SURFACE));
        messages.push_back(createMessage("GPS data age above threshold"));
      }

      // Minimum frequency threshold
      if (hasDouble("frequency") && (getDouble("frequency") < min_frequency_))
      {
        level_ = std::max(level_, static_cast<SafetyLevel::Type>(SafetyLevel::INFORMATIVE));
        messages.push_back(createMessage("frequency below threshold"));
      }
    }

    // Join all messages
    if (!messages.empty())
    {
      message_ = messages[0];
      for (std::size_t i = 1; i < messages.size(); ++i)
        message_ += std::string(". ") + messages[i];
    }
  }

  if ((level_ != SafetyLevel::NONE) && (level_ != SafetyLevel::INFORMATIVE))
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::NAVIGATION_ERROR, true);
}

bool Navigator::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_nav_data_timeout;
  double temp_imu_data_timeout;
  double temp_depth_data_timeout;
  double temp_altitude_data_timeout;
  double temp_dvl_data_timeout;
  double temp_gps_data_timeout;
  double temp_min_frequency;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/nav_data_timeout"), temp_nav_data_timeout);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/imu_data_timeout"), temp_imu_data_timeout);
  ok &=
      cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/depth_data_timeout"), temp_depth_data_timeout);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/altitude_data_timeout"),
                             temp_altitude_data_timeout);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/dvl_data_timeout"), temp_dvl_data_timeout);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/gps_data_timeout"), temp_gps_data_timeout);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/min_frequency"), temp_min_frequency);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  nav_data_timeout_ = temp_nav_data_timeout;
  imu_data_timeout_ = temp_imu_data_timeout;
  depth_data_timeout_ = temp_depth_data_timeout;
  altitude_data_timeout_ = temp_altitude_data_timeout;
  dvl_data_timeout_ = temp_dvl_data_timeout;
  gps_data_timeout_ = temp_gps_data_timeout;
  min_frequency_ = temp_min_frequency;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
