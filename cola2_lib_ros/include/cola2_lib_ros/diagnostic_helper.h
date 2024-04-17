/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Diagnostic helper for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROS_DIAGNOSTIC_HELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROS_DIAGNOSTIC_HELPER_H_

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup DiasgnosticHelper
 * @{
 */

/**
 * @brief Helper class to work with ROS diagnostics. Manages the publishing of the diagnostics, the
 * addition/suppression of diagnostic entries and checks its publishing frequency.
 */
class DiagnosticHelper
{
protected:
  ::ros::Publisher diagnostic_pub_;                  //!< Diagnostic publisher.
  diagnostic_msgs::DiagnosticArray diagnostic_msg_;  //!< Diagnostic message to publish.
  std::set<std::string> messages_;                   //!< Set of messages.
  bool set_level_called_;                            //!< Level has been set.
  bool enabled_;                                     //!< Enabled.
  double last_data_;                                 //!< Last data.
  double last_valid_data_;                           //!< Last valid data.
  std::deque<double> frequency_buffer_;              //!< Frequency buffer.
  std::mutex mtx_;                                   //!< Mutex.

  // Frequency computation parameters
  double frequency_buffer_time_limit_;     //!< Frequency buffer time limit.
  std::size_t frequency_buffer_min_data_;  //!< Frequency buffer min data.
  double frequency_buffer_min_time_;       //!< Frequency buffer min time.

  // Methods
  /**
   * @brief Converts bool to string.
   *
   * @param value Boolean value to convert.
   * @return Returns a string containing "true" or "false".
   */
  std::string boolToString(const bool value) const;

  /**
   * @brief This method reduces the internal frequency buffer size if needed.
   *
   * @param now Current time.
   */
  void shrinkFrequencyBuffer(const double now);

  /**
   * @brief This method uses the frequency buffer to compute the frequency of the reported data.
   *
   * @param now Current time.
   * @return Returns the frequency. If the computation failed, it returns -1.0.
   */
  double computeFrequency(const double now);

  /**
   * @brief Internal implementation of the remove key method.
   *
   * @param key Key to remove.
   */
  void removeKeyValueImpl(const std::string& key);

  /**
   * @brief Internal implementation of the add key method.
   *
   * @param key Key.
   * @param value Value associated to the provided key.
   */
  void addKeyValueImpl(const std::string& key, const std::string& value);

public:
  /**
   * @brief Constructor of the class.
   *
   * @param nh Node handle reference.
   * @param name Name of the diagnostic (typically a standard name, e.g. "dvl").
   * @param hardware_id Name of the hardware (typically the name of the sensor, e.g. "teledyne_rdi_dvl").
   */
  DiagnosticHelper(::ros::NodeHandle& nh, const std::string& name, const std::string& hardware_id);

  /**
   * @brief This method is used to set the frequency buffer time limit.
   *
   * @param frequency_buffer_time_limit Time limit.
   */
  void setFrequencyBufferTimeLimit(const double frequency_buffer_time_limit);

  /**
   * @brief This method is used to set the frequency buffer minimum data amount.
   *
   * @param frequency_buffer_min_data Minimum amount of data.
   */
  void setFrequencyBufferMinData(const std::size_t frequency_buffer_min_data);

  /**
   * @brief This method is used to set the frequency buffer minimum time.
   *
   * @param frequency_buffer_min_time Minimum time.
   */
  void setFrequencyBufferMinTime(const double frequency_buffer_min_time);

  /**
   * @brief This method is used to report whether the node is enabled or disabled.
   *
   * @param enabled Value of the enable flag.
   */
  void setEnabled(const bool enabled);

  /**
   * @brief Report data.
   *
   * @param stamp ROS time of the reported data.
   */
  void reportData(const ::ros::Time stamp = ::ros::Time());  // When the checksum is ok

  /**
   * @brief Report valid data. This provides extra information, as the data has been validated by the node.
   *
   * @param stamp ROS time of the reported valid data.
   */
  void reportValidData(const ::ros::Time stamp = ::ros::Time());  // When the data can be used

  /**
   * @brief Add key and the corresponding boolean value.
   *
   * @param key Key.
   * @param value Boolean value associated to the provided key.
   */
  void addKeyValue(const std::string& key, const bool value);

  /**
   * @brief Add key and the corresponding double value.
   *
   * @param key Key.
   * @param value Double value associated to the provided key.
   */
  void addKeyValue(const std::string& key, const double value);

  /**
   * @brief Add key and the corresponding integer value.
   *
   * @param key Key.
   * @param value Integer value associated to the provided key.
   */
  void addKeyValue(const std::string& key, const int value);

  /**
   * @brief Add key and the corresponding char array value.
   *
   * @param key Key.
   * @param value Char array value associated to the provided key.
   */
  void addKeyValue(const std::string& key, const char* value);

  /**
   * @brief Add key and the corresponding string value.
   *
   * @param key Key.
   * @param value String value associated to the provided key.
   */
  void addKeyValue(const std::string& key, const std::string& value);

  /**
   * @brief Remove a key and its corresponding value.
   *
   * @param key Key to remove.
   */
  void removeKeyValue(const std::string& key);

  /**
   * @brief Set level with an optional message.
   *
   * @param level Level.
   * @param message Optional message.
   */
  void setLevelAndMessage(const std::uint8_t level, const std::string& message = "");

  /**
   * @brief Set message.

   * @param message Message.
   */
  void setMessage(const std::string& message);

  /**
   * @brief Publish diagnostic.
   *
   * @param stamp Optional ROS time corresponding to the time of the published data.
   */
  void publish(const ::ros::Time stamp = ::ros::Time());
};
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROS_DIAGNOSTIC_HELPER_H_
