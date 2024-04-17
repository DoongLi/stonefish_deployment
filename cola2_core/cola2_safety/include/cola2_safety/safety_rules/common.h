/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMON_H_
#define COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMON_H_

#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/serviceclient_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <cola2_msgs/Setpoints.h>
#include <cola2_msgs/WorldWaypointReq.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

/**
 * \brief This namespace contains everything related to safety rules
 */
namespace SafetyRules
{
/**
 * \brief This namespace contains the bit definitions for the status code
 */
namespace StatusCodeBits
{
const std::size_t BCB_STATUS_INDICATOR_2 = 20;
const std::size_t BCB_STATUS_INDICATOR_1 = 19;
const std::size_t RA_DROP_WEIGHT = 18;
const std::size_t RA_EMERGENCY_SURFACE = 17;
const std::size_t RA_ABORT_SURFACE = 16;
const std::size_t RA_ABORT = 15;
const std::size_t LOW_BATTERY_WARNING = 14;
const std::size_t LOW_BATTERY_ERROR = 13;
const std::size_t WATCHDOG_TIMER = 12;
const std::size_t NAVIGATION_ERROR = 11;
const std::size_t NO_ALTITUDE_ERROR = 10;
const std::size_t WATER_INSIDE = 9;
const std::size_t HIGH_TEMPERATURE = 8;
const std::size_t CURRENT_MISSION_STEPS_BASE = 7;  // From 0 to 7

/**
 * \brief This function can be used to set a bit of the status code to either true or false
 * \param[out] Status code
 * \param[in] Bit that needs to be set
 * \param[in] State of the bit to be set
 */
void setBit(std::uint32_t*, const std::size_t, const bool);

/**
 * \brief This function can be used to retrieve the state of a bit in the status code
 * \param[in] Status code
 * \param[in] Requested bit
 * \return State of the requested bit
 */
bool getBit(const std::uint32_t, const std::size_t);
}  // namespace StatusCodeBits

/**
 * \brief This class is used to represent a safety level
 */
struct SafetyLevel
{
  using Type = std::size_t;
  static const std::size_t NONE = 0;
  static const std::size_t INFORMATIVE = 1;
  static const std::size_t ABORT = 2;
  static const std::size_t ABORT_AND_SURFACE = 3;
  static const std::size_t EMERGENCY_SURFACE = 4;
  static const std::size_t DROP_WEIGHT = 5;

  /**
   * \brief This method returns a printable name for each safety level
   * \param[in] Safety level
   * \return Printable name for the given safety level
   */
  static std::string getName(const std::size_t);
};

/**
 * \brief This class presents the interface of a safety rule
 */
class SafetyRuleBaseClass
{
protected:
  // The DataType class is used to represent the types that can be parsed from the diagnostics messages
  enum class DataType
  {
    Bool,
    Double,
    Int,
    String
  };

  // ParseList type. It is a vector of status_name, key_name, var_name and data_type
  using ParseList = std::vector<std::tuple<std::string, std::string, std::string, DataType>>;

  // ParseList type. It is a vector of status_name, hardware_id, key_name, var_name and data_type
  using ParseListWithHwId = std::vector<std::tuple<std::string, std::string, std::string, std::string, DataType>>;

  const std::string rule_name_;
  ParseListWithHwId parse_list_;
  std::size_t level_;
  std::string message_;
  double last_diagnostic_;
  double last_valid_diagnostics_data_;
  double last_valid_config_;

  // These maps contain the parsed variables from the last diagnostic message received
  std::map<std::string, bool> bool_vars_;
  std::map<std::string, double> double_vars_;
  std::map<std::string, int> int_vars_;
  std::map<std::string, std::string> string_vars_;

  /**
   * \brief This method returns a printable string name for a given DataType
   * \param[in] Requested DataType
   * \return Printable string name for the given DataType
   */
  static std::string dataTypeToString(const DataType&);

  /**
   * \brief This method appends the rule name before the given string to create a message
   * \param[in] Input string
   * \return Message string
   */
  std::string createMessage(const std::string&);

public:
  // These are common values that are used throughout the implementation of different safety rules
  static constexpr double INIT_TIME = 20.0;
  static constexpr double NO_DIAGNOSTICS_TIME = 30.0;
  static constexpr double NO_DIAGNOSTICS_ESCALATED_TIME = 630.0;

  // These are strings with special meaning
  inline static const std::string DIAGNOSTIC_STATUS_LEVEL = "LeVeL__LeVeL__LeVeL";
  inline static const std::string DIAGNOSTIC_STATUS_MESSAGE = "MeSsAgE__MeSsAgE__MeSsAgE";

  /**
   * \brief Constructor
   * \param[in] Rule name
   */
  explicit SafetyRuleBaseClass(const std::string&);

  /**
   * \brief Destructor
   */
  virtual ~SafetyRuleBaseClass();

  /**
   * \brief This method can be used to set the ParseList. The ParseList defines which elements must be parsed from
   *        the diagnostic messages
   * \param[in] The ParseList
   */
  virtual void setParseList(const ParseList&);

  /**
   * \brief This method can be used to set the ParseListWithHwId. The ParseListWithHwId defines which elements must
   *        be parsed from the diagnostic messages
   * \param[in] The ParseListWithHwId
   */
  virtual void setParseListWithHwId(const ParseListWithHwId&);

  /**
   * \brief This method is called by the safety supervisor, and it is used to provide the most recent diagnostic
   *        message to the safety rule
   * \param[in] The most recent diagnostic array received
   */
  virtual void diagnosticsUpdate(const diagnostic_msgs::DiagnosticArray&);

  /**
   * \brief This is an optional method that is called automatically right after the diagnosticsUpdate method is
   *        completed. If reimplemented by the safety rule, it can be used to perform further actions such as taking
   *        the parsed diagnostic message to put its data to specific members of the safety rule
   */
  virtual void parseDiagnostics();

  /**
   * \brief This method is used to obtain a parsed boolean variable given its name
   * \param[in] Boolean variable name
   * \return The value of the requested boolean variable
   */
  virtual bool getBool(const std::string&);

  /**
   * \brief This method is used to check if a boolean variable has been parsed from the last diagnostic message
   * \param[in] Boolean variable name
   * \return Returns true if the variable is available and false otherwise
   */
  virtual bool hasBool(const std::string&);

  /**
   * \brief This method is used to obtain a parsed floating point variable given its name
   * \param[in] Floating point variable name
   * \return The value of the requested floating point variable
   */
  virtual double getDouble(const std::string&);

  /**
   * \brief This method is used to check if a floating point variable has been parsed from the last diagnostic message
   * \param[in] Floating point variable name
   * \return Returns true if the variable is available and false otherwise
   */
  virtual bool hasDouble(const std::string&);

  /**
   * \brief This method is used to obtain a parsed integer variable given its name
   * \param[in] Integer variable name
   * \return The value of the requested integer variable
   */
  virtual int getInt(const std::string&);

  /**
   * \brief This method is used to check if an integer variable has been parsed from the last diagnostic message
   * \param[in] Integer variable name
   * \returns true if the variable is available and false otherwise
   */
  virtual bool hasInt(const std::string&);

  /**
   * \brief This method is used to obtain a parsed string variable given its name
   * \param[in] String variable name
   * \return The value of the requested string variable
   */
  virtual std::string getString(const std::string&);

  /**
   * \brief This method is used to check if a string variable has been parsed from the last diagnostic message
   * \param[in] String variable name
   * \returns true if the variable is available and false otherwise
   */
  virtual bool hasString(const std::string&);

  /**
   * \brief This method is periodically called from the safety supervisor to update the internal state of the
   *        rule, such as its safety level and message, and to compute the status code
   * \param[in] ROS time stamp
   * \param[out] Status code to be updated
   */
  virtual void periodicUpdate(const ros::Time&, std::uint32_t*);

  /**
   * \brief This method must be reimplemented to obtain configuration from the ROS param server
   * \return Returns true if the parameters were successfully loaded and false otherwise
   */
  virtual bool loadConfigFromParamServer();

  /**
   * \brief This method returns the rule name
   * \return Rule name
   */
  virtual std::string getRuleName() const;

  /**
   * \brief This method returns the rule safety level
   * \return Rule safety level
   */
  virtual std::size_t getLevel() const;

  /**
   * \brief This method is used to obtain the message associated with the current safety level of the rule. It provides
   *        a description of the conditions that triggered the current safety level
   * \return Message string associated with the current safety level of the rule
   */
  virtual std::string getMessage() const;

  /**
   * \brief This method converts a string to a boolean variable
   * \param[in] Boolean string to be converted
   * \return Boolean
   */
  static bool stringToBool(const std::string&);
};
}  // namespace SafetyRules

#endif  // COLA2_SAFETY_INCLUDE_COLA2_SAFETY_SAFETY_RULES_COMMON_H_
