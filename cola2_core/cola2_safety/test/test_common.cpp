/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>

class Common : public SafetyRules::SafetyRuleBaseClass
{
public:
  double a;
  int b;
  bool c;
  std::string d;
  Common();
  void parseDiagnostics();
  bool checkDataTypeNames();
};

Common::Common() : SafetyRules::SafetyRuleBaseClass("common_safety_rule"), a(0.0), b(0), c(false)
{
  message_ = "Test";

  const ParseListWithHwId parse_list({ { "/test", "*", "a", "a", DataType::Double },
                                       { "/test", "*", "b", "b", DataType::Int },
                                       { "/test", "*", "c", "c", DataType::Bool },
                                       { "/test", "wrong_hw_id", "c", "c", DataType::Bool },
                                       { "/test", "*", "d", "d", DataType::String },
                                       { "/test", "*", DIAGNOSTIC_STATUS_LEVEL, "level_b", DataType::Bool },
                                       { "/test", "*", DIAGNOSTIC_STATUS_LEVEL, "level_d", DataType::Double },
                                       { "/test", "*", DIAGNOSTIC_STATUS_LEVEL, "level_i", DataType::Int },
                                       { "/test", "*", DIAGNOSTIC_STATUS_LEVEL, "level_s", DataType::String },
                                       { "/test", "*", DIAGNOSTIC_STATUS_MESSAGE, "message_b", DataType::Bool },
                                       { "/test", "*", DIAGNOSTIC_STATUS_MESSAGE, "message_d", DataType::Double },
                                       { "/test", "*", DIAGNOSTIC_STATUS_MESSAGE, "message_i", DataType::Int },
                                       { "/test", "*", DIAGNOSTIC_STATUS_MESSAGE, "message_s", DataType::String } });
  setParseListWithHwId(parse_list);
}

void Common::parseDiagnostics()
{
  if (hasDouble("a"))
    a = getDouble("a");
  if (hasInt("b"))
    b = getInt("b");
  if (hasBool("c"))
    c = getBool("c");
  if (hasString("d"))
    d = getString("d");
}

bool Common::checkDataTypeNames()
{
  bool ok = true;
  if (dataTypeToString(DataType::Double).compare("double") != 0)
    ok = false;
  if (dataTypeToString(DataType::Int).compare("int") != 0)
    ok = false;
  if (dataTypeToString(DataType::Bool).compare("bool") != 0)
    ok = false;
  if (dataTypeToString(DataType::String).compare("string") != 0)
    ok = false;
  if (dataTypeToString(static_cast<DataType>(100)).compare("unknown") != 0)
    ok = false;
  return ok;
}

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create error codes message
  std::string error_codes;

  // Test status bits
  std::uint32_t status_code = 0;
  bool bit_state = SafetyRules::StatusCodeBits::getBit(status_code, 1);
  if (bit_state)
    error_codes += "01 ";
  SafetyRules::StatusCodeBits::setBit(&status_code, 1, true);
  bit_state = SafetyRules::StatusCodeBits::getBit(status_code, 1);
  if (!bit_state)
    error_codes += "02 ";
  if (status_code != 2)
    error_codes += "03 ";
  SafetyRules::StatusCodeBits::setBit(&status_code, 1, false);
  if (status_code != 0)
    error_codes += "04 ";

  // Test safety level names
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::NONE).compare("NONE") != 0)
    error_codes += "05 ";
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::INFORMATIVE).compare("INFORMATIVE") != 0)
    error_codes += "06 ";
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::ABORT).compare("ABORT") != 0)
    error_codes += "07 ";
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::ABORT_AND_SURFACE).compare("ABORT_AND_SURFACE") != 0)
    error_codes += "08 ";
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::EMERGENCY_SURFACE).compare("EMERGENCY_SURFACE") != 0)
    error_codes += "09 ";
  if (SafetyRules::SafetyLevel::getName(SafetyRules::SafetyLevel::DROP_WEIGHT).compare("DROP_WEIGHT") != 0)
    error_codes += "10 ";
  if (SafetyRules::SafetyLevel::getName(100).compare("UNKNOWN_SAFETY_LEVEL") != 0)
    error_codes += "11 ";

  // Create fake diagnostic msg
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic_status;
  diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::STALE;  // This is wrong on purpose
  diagnostic_status.name = "/tEsT";                                    // This is wrong on purpose
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "a";
  diagnostic_key_value.value = "1.0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "b";
  diagnostic_key_value.value = "1";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "c";
  diagnostic_key_value.value = "tRuE";  // This is wrong on purpose
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "d";
  diagnostic_key_value.value = "tEsT";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Test rare cases in safety rule base class
  Common common;
  common.diagnosticsUpdate(diagnostic_array);
  if (common.a != 0.0)
    error_codes += "12 ";
  common.loadConfigFromParamServer();
  common.diagnosticsUpdate(diagnostic_array);
  if (common.a != 0.0)
    error_codes += "13 ";
  diagnostic_array.status[0].name = "/test";
  common.diagnosticsUpdate(diagnostic_array);
  if (common.a != 0.0)
    error_codes += "14 ";
  diagnostic_array.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
  common.diagnosticsUpdate(diagnostic_array);
  if (common.a != 1.0)
  {
    ROS_FATAL_STREAM("a = " << common.a);
    error_codes += "15 ";
  }
  if (common.c)
    error_codes += "16 ";
  diagnostic_array.status[0].values[2].value = "True";
  common.diagnosticsUpdate(diagnostic_array);
  if (!common.c)
    error_codes += "17 ";
  if (common.d != "tEsT")
    error_codes += "18 ";

  // Test other small methods
  if (common.getRuleName().compare("common_safety_rule") != 0)
    error_codes += "19 ";
  if (common.getMessage().compare("Test") != 0)
    error_codes += "20 ";
  if (!common.checkDataTypeNames())
    error_codes += "21 ";

  // These get methods should throw
  try
  {
    common.getDouble("e");
    error_codes += "22 ";
  }
  catch (...)
  {
  }
  try
  {
    common.getInt("e");
    error_codes += "23 ";
  }
  catch (...)
  {
  }
  try
  {
    common.getBool("e");
    error_codes += "24 ";
  }
  catch (...)
  {
  }
  try
  {
    common.getString("e");
    error_codes += "25 ";
  }
  catch (...)
  {
  }

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  Common* common_ptr = new Common();
  delete common_ptr;
  SafetyRules::SafetyRuleBaseClass* base_class_ptr = new SafetyRules::SafetyRuleBaseClass("common_safety_rule");
  base_class_ptr->parseDiagnostics();  // Call base class parseDiagnostics() method
  delete base_class_ptr;

  if (!error_codes.empty())
  {
    ROS_FATAL_STREAM("Test failed with error codes: " << error_codes);
    FAIL();
  }
}

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
    ros::console::notifyLoggerLevelsChanged();
  ros::init(argc, argv, "test_common");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
