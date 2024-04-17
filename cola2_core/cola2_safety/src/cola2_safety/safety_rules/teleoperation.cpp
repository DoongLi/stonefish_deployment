/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/teleoperation.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
Teleoperation::Teleoperation(const std::string& rule_name, ros::NodeHandle* nh_ptr)
  : SafetyRuleBaseClass(rule_name)
  , last_ack_(0.0)
  , teleoperation_link_timeout_(0.0)
  , last_autonomous_(ros::Time::now().toSec())
  , last_captain_status_(last_autonomous_)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/control/") + rule_name_, "enabled", "enabled", DataType::Bool},
    {std::string("/control/") + rule_name_, "last_ack", "last_ack", DataType::Double}
  });
  setParseList(parse_list);

  // Subscriber
  sub_captain_ = nh_ptr->subscribe(cola2::ros::getNamespace() + "/captain/captain_status", 1,
                                   &Teleoperation::captainStatusCallback, this);
}

void
Teleoperation::parseDiagnostics()
{
  if (hasBool("enabled"))
    last_valid_diagnostics_data_ = last_diagnostic_;
}

void
Teleoperation::captainStatusCallback(const cola2_msgs::CaptainStatus& msg)
{
  last_captain_status_ = msg.header.stamp.toSec();
  if (msg.state != cola2_msgs::CaptainStatus::IDLE)
    last_autonomous_ = last_captain_status_;
}

void
Teleoperation::periodicUpdate(const ros::Time& stamp, std::uint32_t*)
{
  // Clear level and message
  level_ = SafetyLevel::NONE;
  message_.clear();

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
  else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_ESCALATED_TIME)
  {
    level_ = SafetyLevel::EMERGENCY_SURFACE;
    message_ = createMessage("too much time without valid diagnostics, escalated to emergency");
  }
  else if (stamp.toSec() - last_valid_diagnostics_data_ > NO_DIAGNOSTICS_TIME)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("too much time without valid diagnostics");
  }
  else if (!(hasDouble("last_ack") && getDouble("last_ack") < teleoperation_link_timeout_))
  {
    if ((stamp.toSec() - last_autonomous_ > teleoperation_link_timeout_) &&
        (stamp.toSec() - last_captain_status_ <= NO_DIAGNOSTICS_TIME) &&
        (stamp.toSec() - last_valid_config_ >= INIT_TIME))
    {
      message_ = createMessage("teleoperation link lost without autonomous mode active");
      level_ = SafetyLevel::ABORT_AND_SURFACE;
    }
  }
  else if (stamp.toSec() - last_captain_status_ > NO_DIAGNOSTICS_TIME)  // Reuse no diagnostics time
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("captain status is not being published");
  }
}

bool
Teleoperation::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_teleoperation_link_timeout;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/teleoperation_link_timeout"), temp_teleoperation_link_timeout);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  teleoperation_link_timeout_ = temp_teleoperation_link_timeout;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
