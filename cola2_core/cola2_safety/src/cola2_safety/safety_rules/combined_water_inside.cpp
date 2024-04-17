/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/combined_water_inside.h>

namespace SafetyRules
{
  CombinedWaterInside::CombinedWaterInside(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , check_battery_housing_(true)
  , check_extra_housing_(true)
  , water_battery_external_(false)
  , water_battery_internal_(false)
  , water_main_external_(false)
  , water_main_internal_(false)
  , water_extra_(false)
{
  loadConfigFromParamServer();
}

void
CombinedWaterInside::periodicUpdate(const ros::Time& stamp, std::uint32_t* status_code_ptr)
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
  else if ((!check_battery_housing_) && water_main_internal_)  // If only checking the main housing
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("water inside");
  }
  else
  {
    //         DIAGNOSTICS FROM THE WATER SENSORS                      INTERPRETATION
    //  ___________________________________________________      ___________________________
    // |       |           |           |         |         |    |       |     |     |       |
    // | EXTRA | MAIN_INT  | MAIN_EXT  | BAT_INT | BAT_EXT |    | MAIN  | BAT | INS | FAIL  |
    // |_______|___________|___________|_________|_________|    |_______|_____|_____|_______|
    // |       |           |           |         |         |    |       |     |     |       |
    // |   0   |     0     |     0     |    0    |    0    |    |   0   |  0  |  0  |   0   |
    // |   0   |     0     |     0     |    0    |    1    |    |   -   |  -  |  -  |   1   |
    // |   0   |     0     |     0     |    1    |    0    |    |   -   |  -  |  -  |   1   |
    // |   0   |     0     |     0     |    1    |    1    |    |   -   |  -  |  -  |   1   |
    // |   0   |     0     |     1     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   0   |     0     |     1     |    0    |    1    |    |   -   |  -  |  -  |   1   |
    // |   0   |     0     |     1     |    1    |    0    |    |   0   |  1  |  0  |   0   |
    // |   0   |     0     |     1     |    1    |    1    |    |   -   |  -  |  -  |   1   |
    // |   0   |     1     |     0     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   0   |     1     |     0     |    0    |    1    |    |   1   |  0  |  0  |   0   |
    // |   0   |     1     |     0     |    1    |    0    |    |   -   |  -  |  -  |   1   |
    // |   0   |     1     |     0     |    1    |    1    |    |   1   |  1  |  0  |   0   |
    // |   0   |     1     |     1     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   0   |     1     |     1     |    0    |    1    |    |   -   |  -  |  -  |   1   |
    // |   0   |     1     |     1     |    1    |    0    |    |   1   |  1  |  0  |   0   |
    // |   0   |     1     |     1     |    1    |    1    | -> |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     0     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     0     |    0    |    1    |    |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     0     |    1    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     0     |    1    |    1    |    |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     1     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     0     |     1     |    0    |    1    |    |   0   |  0  |  1  |   0   |
    // |   1   |     0     |     1     |    1    |    0    |    |   0   |  1  |  1  |   0   |
    // |   1   |     0     |     1     |    1    |    1    |    |   0   |  1  |  1  |   0   |
    // |   1   |     1     |     0     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     1     |     0     |    0    |    1    |    |   1   |  0  |  1  |   0   |
    // |   1   |     1     |     0     |    1    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     1     |     0     |    1    |    1    |    |   1   |  1  |  1  |   0   |
    // |   1   |     1     |     1     |    0    |    0    |    |   -   |  -  |  -  |   1   |
    // |   1   |     1     |     1     |    0    |    1    |    |   1   |  0  |  1  |   0   |
    // |   1   |     1     |     1     |    1    |    0    |    |   1   |  1  |  1  |   0   |
    // |   1   |     1     |     1     |    1    |    1    |    |   1   |  1  |  1  |   0   |
    // |_______|___________|___________|_________|_________|    |_______|_____|_____|_______|

    // Create masks from the columns of the interpretation table. First row goes to the right (first bit)
    const std::uint32_t main_mask    = 0b11101010000000000100101000000000;
    const std::uint32_t battery_mask = 0b11001000110000000100100001000000;
    const std::uint32_t extra_mask   = 0b11101010111000000000000000000000;
    const std::uint32_t failure_mask = 0b00010101000111111011010110111110;

    // Compute row from the table
    const std::size_t row = (water_battery_external_ ? 1 : 0) +
                            (water_battery_internal_ ? 2 : 0) +
                            (water_main_external_ ? 4 : 0) +
                            (water_main_internal_ ? 8 : 0) +
                            (water_extra_ ? 16 : 0);

    // Use masks to detect where the water is
    if (failure_mask & (1 << row))
    {
      message_ = createMessage("water inside unknown housing");
      level_ = SafetyLevel::ABORT_AND_SURFACE;
    }
    else
    {
      // Create list of housings with water
      std::vector<std::string> housings_with_water;
      if (main_mask & (1 << row))
        housings_with_water.push_back("main");
      if (battery_mask & (1 << row))
        housings_with_water.push_back("batteries");
      if (extra_mask & (1 << row))
        housings_with_water.push_back("extra");

      // If there are housings with water
      if (!housings_with_water.empty())
      {
        level_ = SafetyLevel::ABORT_AND_SURFACE;

        // Create human readable message
        message_ = createMessage("water inside ") + housings_with_water[0];
        if (housings_with_water.size() == 2)
          message_ += std::string(" and ") + housings_with_water[1];
        else if (housings_with_water.size() == 3)
          message_ += std::string(", ") + housings_with_water[1] + std::string(" and ") + housings_with_water[2];
        if (housings_with_water.size() == 1)
          message_ += std::string(" housing");
        else
          message_ += std::string(" housings");
      }
    }
  }

  if ((level_ != SafetyLevel::NONE) && (level_ != SafetyLevel::INFORMATIVE))
    StatusCodeBits::setBit(status_code_ptr, StatusCodeBits::WATER_INSIDE, true);
}

bool
CombinedWaterInside::loadConfigFromParamServer()
{
  // Load config from param server
  bool temp_check_battery_housing;
  bool temp_check_extra_housing;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/check_battery_housing"), temp_check_battery_housing);
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/check_extra_housing"), temp_check_extra_housing);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  check_battery_housing_ = temp_check_battery_housing;
  check_extra_housing_ = temp_check_extra_housing;
  last_valid_config_ = ros::Time::now().toSec();

  // As if the rule was reset
  water_battery_external_ = false;
  water_battery_internal_ = false;
  water_main_external_ = false;
  water_main_internal_ = false;
  water_extra_ = false;
  updateParseList();
  return ok;
}

void
CombinedWaterInside::updateParseList()
{
  if (check_battery_housing_)
  {
    ParseList parse_list({
      {"/safety/main_control_board", "water_internal_inside", "water_main_internal", DataType::Bool},
      {"/safety/main_control_board", "water_external_inside", "water_main_external", DataType::Bool},
      {"/safety/battery_control_board", "water_internal_inside", "water_battery_internal", DataType::Bool},
      {"/safety/battery_control_board", "water_external_inside", "water_battery_external", DataType::Bool}
    });
    if (check_extra_housing_)
      parse_list.push_back({"/safety/extra_housing", "water_inside", "water_extra", DataType::Bool});
    setParseList(parse_list);
  }
  else
  {
    const ParseList parse_list({  // If only one housing, we use the battery control board
      {"/safety/battery_control_board", "water_internal_inside", "water_main_internal", DataType::Bool}
    });
    setParseList(parse_list);
  }
}

void
CombinedWaterInside::parseDiagnostics()
{
  if (check_battery_housing_)
  {
    const bool valid_diagnostic = hasBool("water_main_internal") &&
                                  hasBool("water_main_external") &&
                                  hasBool("water_battery_internal") &&
                                  hasBool("water_battery_external") &&
                                  (check_extra_housing_ ? hasBool("water_extra") : true);
    if (valid_diagnostic)
    {
      last_valid_diagnostics_data_ = last_diagnostic_;
      water_main_internal_ = getBool("water_main_internal");
      water_main_external_ = getBool("water_main_external");
      water_battery_internal_ = getBool("water_battery_internal");
      water_battery_external_ = getBool("water_battery_external");
      if (check_extra_housing_)
        water_extra_ = getBool("water_extra");
    }
  }
  else
  {
    if (hasBool("water_main_internal"))
    {
      last_valid_diagnostics_data_ = last_diagnostic_;
      water_main_internal_ = getBool("water_main_internal");
    }
  }
}
}  // namespace SafetyRules
