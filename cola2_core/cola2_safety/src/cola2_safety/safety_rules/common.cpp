/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>

#include <exception>
#include <set>
#include <stdexcept>

namespace SafetyRules
{
namespace StatusCodeBits
{
void setBit(std::uint32_t* status_code_ptr, const std::size_t bit, const bool state)
{
  if (state)
  {
    (*status_code_ptr) |= (1 << bit);
  }
  else
  {
    (*status_code_ptr) &= ~(1 << bit);
  }
}

bool getBit(const std::uint32_t status_code, const std::size_t bit)
{
  return status_code & (1 << bit);
}
}  // namespace StatusCodeBits

std::string SafetyLevel::getName(const std::size_t level)
{
  // Implemented as a static method instead of a fixed length array of
  // strings to avoid crashes if the level is unknown (out of bounds)
  switch (level)
  {
    case NONE:
      return std::string("NONE");
    case INFORMATIVE:
      return std::string("INFORMATIVE");
    case ABORT:
      return std::string("ABORT");
    case ABORT_AND_SURFACE:
      return std::string("ABORT_AND_SURFACE");
    case EMERGENCY_SURFACE:
      return std::string("EMERGENCY_SURFACE");
    case DROP_WEIGHT:
      return std::string("DROP_WEIGHT");
    default:
      break;
  }
  return std::string("UNKNOWN_SAFETY_LEVEL");
}

std::string SafetyRuleBaseClass::dataTypeToString(const DataType& data_type)
{
  switch (data_type)
  {
    case DataType::Bool:
      return std::string("bool");
    case DataType::Double:
      return std::string("double");
    case DataType::Int:
      return std::string("int");
    case DataType::String:
      return std::string("string");
    default:
      break;
  }
  return std::string("unknown");
}

bool SafetyRuleBaseClass::stringToBool(const std::string& data)
{
  if ((data.compare("True") == 0) || (data.compare("true") == 0))
  {
    return true;
  }
  if ((data.compare("False") != 0) && (data.compare("false") != 0))
  {
    throw std::runtime_error(std::string("Invalid boolean string: ") + data);
  }
  return false;
}

std::string SafetyRuleBaseClass::createMessage(const std::string& input_message)
{
  return std::string("(") + rule_name_ + std::string("): ") + input_message;
}

SafetyRuleBaseClass::SafetyRuleBaseClass(const std::string& rule_name)
  : rule_name_(rule_name)
  , level_(SafetyLevel::NONE)
  , last_diagnostic_(0.0)
  , last_valid_diagnostics_data_(0.0)
  , last_valid_config_(0.0)
{
}

SafetyRuleBaseClass::~SafetyRuleBaseClass()
{
}

void SafetyRuleBaseClass::setParseList(const ParseList& parse_list)
{
  ParseListWithHwId parse_list_with_hw_id;
  parse_list_with_hw_id.reserve(parse_list.size());
  for (const auto& elem : parse_list)
  {
    parse_list_with_hw_id.push_back(
        { std::get<0>(elem), "*", std::get<1>(elem), std::get<2>(elem), std::get<3>(elem) });
  }
  parse_list_ = parse_list_with_hw_id;
}

void SafetyRuleBaseClass::setParseListWithHwId(const ParseListWithHwId& parse_list)
{
  parse_list_ = parse_list;
}

void SafetyRuleBaseClass::diagnosticsUpdate(const diagnostic_msgs::DiagnosticArray& msg)
{
  // Store time
  last_diagnostic_ = msg.header.stamp.toSec();

  // Clear maps
  bool_vars_.clear();
  double_vars_.clear();
  int_vars_.clear();

  // Parse
  std::set<std::string> stale;
  for (const auto& elem : parse_list_)
  {
    for (const auto& diagnostic_status : msg.status)
    {
      // If it is not the matching element, continue
      if (diagnostic_status.name.compare(std::get<0>(elem)) != 0)
      {
        continue;
      }
      if ((std::get<1>(elem).compare("*") != 0) && (diagnostic_status.hardware_id.compare(std::get<1>(elem)) != 0))
      {
        continue;
      }

      // Check for STALE
      if (diagnostic_status.level == diagnostic_msgs::DiagnosticStatus::STALE)
      {
        stale.insert(diagnostic_status.name);
        continue;
      }

      // Check if the user directly wants the level of the DiagnosticStatus
      if (DIAGNOSTIC_STATUS_LEVEL.compare(std::get<2>(elem)) == 0)
      {
        // Parse level according to the type
        if (std::get<4>(elem) == DataType::Bool)
        {
          bool_vars_.insert({ std::get<3>(elem), static_cast<bool>(diagnostic_status.level) });
        }
        else if (std::get<4>(elem) == DataType::Double)
        {
          double_vars_.insert({ std::get<3>(elem), static_cast<double>(diagnostic_status.level) });
        }
        else if (std::get<4>(elem) == DataType::Int)
        {
          int_vars_.insert({ std::get<3>(elem), static_cast<int>(diagnostic_status.level) });
        }
        else
        {
          string_vars_.insert({ std::get<3>(elem), std::to_string(diagnostic_status.level) });
        }
        continue;
      }

      // Check if the user directly wants the message of the DiagnosticStatus
      if (DIAGNOSTIC_STATUS_MESSAGE.compare(std::get<2>(elem)) == 0)
      {
        try
        {
          // Parse level according to the type
          if (std::get<4>(elem) == DataType::Bool)
          {
            bool_vars_.insert({ std::get<3>(elem), stringToBool(diagnostic_status.message) });
          }
          else if (std::get<4>(elem) == DataType::Double)
          {
            double_vars_.insert({ std::get<3>(elem), std::stod(diagnostic_status.message) });
          }
          else if (std::get<4>(elem) == DataType::Int)
          {
            int_vars_.insert({ std::get<3>(elem), std::stoi(diagnostic_status.message) });
          }
          else
          {
            string_vars_.insert({ std::get<3>(elem), diagnostic_status.message });
          }
        }
        catch (const std::exception&)
        {
          ROS_WARN_STREAM("(" << rule_name_ << "): unable to parse 'message' of type "
                              << dataTypeToString(std::get<4>(elem))
                              << " from diagnostic: " << diagnostic_status.message);
        }
        continue;
      }

      // Check KeyValues
      for (const auto& key_value : diagnostic_status.values)
      {
        if (key_value.key.compare(std::get<2>(elem)) == 0)
        {
          try
          {
            // Parse the value according to the type
            if (std::get<4>(elem) == DataType::Bool)
            {
              bool_vars_.insert({ std::get<3>(elem), stringToBool(key_value.value) });
            }
            else if (std::get<4>(elem) == DataType::Double)
            {
              double_vars_.insert({ std::get<3>(elem), std::stod(key_value.value) });
            }
            else if (std::get<4>(elem) == DataType::Int)
            {
              int_vars_.insert({ std::get<3>(elem), std::stoi(key_value.value) });
            }
            else
            {
              string_vars_.insert({ std::get<3>(elem), key_value.value });
            }
          }
          catch (const std::exception&)
          {
            ROS_WARN_STREAM("(" << rule_name_ << "): unable to parse " << std::get<2>(elem) << " of type "
                                << dataTypeToString(std::get<4>(elem)) << " from diagnostic: " << key_value.value);
          }
          break;
        }
      }
    }
  }

  // Show warning for STALE diagnostics
  for (const auto& elem : stale)
  {
    ROS_WARN_STREAM("(" << rule_name_ << "): diagnostic " << elem << " is STALE");
  }

  // Call parse diagnostics method
  parseDiagnostics();
}

void SafetyRuleBaseClass::parseDiagnostics()
{
  last_valid_diagnostics_data_ = last_diagnostic_;
}

bool SafetyRuleBaseClass::getBool(const std::string& var_name)
{
  const auto it = bool_vars_.find(var_name);
  if (it == bool_vars_.end())
  {
    throw std::runtime_error(std::string("(") + rule_name_ + std::string("): bool variable not found: ") + var_name);
  }
  return it->second;
}

bool SafetyRuleBaseClass::hasBool(const std::string& var_name)
{
  return bool_vars_.find(var_name) != bool_vars_.end();
}

double SafetyRuleBaseClass::getDouble(const std::string& var_name)
{
  const auto it = double_vars_.find(var_name);
  if (it == double_vars_.end())
  {
    throw std::runtime_error(std::string("(") + rule_name_ + std::string("): double variable not found: ") + var_name);
  }
  return it->second;
}

bool SafetyRuleBaseClass::hasDouble(const std::string& var_name)
{
  return double_vars_.find(var_name) != double_vars_.end();
}

int SafetyRuleBaseClass::getInt(const std::string& var_name)
{
  const auto it = int_vars_.find(var_name);
  if (it == int_vars_.end())
  {
    throw std::runtime_error(std::string("(") + rule_name_ + std::string("): int variable not found: ") + var_name);
  }
  return it->second;
}

bool SafetyRuleBaseClass::hasInt(const std::string& var_name)
{
  return int_vars_.find(var_name) != int_vars_.end();
}

std::string SafetyRuleBaseClass::getString(const std::string& var_name)
{
  const auto it = string_vars_.find(var_name);
  if (it == string_vars_.end())
  {
    throw std::runtime_error(std::string("(") + rule_name_ + std::string("): string variable not found: ") + var_name);
  }
  return it->second;
}

bool SafetyRuleBaseClass::hasString(const std::string& var_name)
{
  return string_vars_.find(var_name) != string_vars_.end();
}

void SafetyRuleBaseClass::periodicUpdate(const ros::Time&, std::uint32_t*)
{
}

bool SafetyRuleBaseClass::loadConfigFromParamServer()
{
  /*ROS_INFO_STREAM(rule_name_ << ": loading nothing from param server (base class)");*/
  last_valid_config_ = ros::Time::now().toSec();
  return true;
}

std::string SafetyRuleBaseClass::getRuleName() const
{
  return rule_name_;
}

std::size_t SafetyRuleBaseClass::getLevel() const
{
  return level_;
}

std::string SafetyRuleBaseClass::getMessage() const
{
  return message_;
}
}  // namespace SafetyRules
