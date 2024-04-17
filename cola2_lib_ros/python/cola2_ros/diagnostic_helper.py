#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to create diagnostic helper to ease the task of publishing diagnostics.
"""

from threading import Lock
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticArray


class DiagnosticHelper(object):
    """
    Helper class to work with ROS diagnostics. Manages the publishing of the diagnostics, the addition/suppression of diagnostic
    entries and checks its publishing frequency.

    :param name: Name of the diagnostic (typically a standard name, e.g. "dvl").
    :type name: str
    :param hardware_id: Name of the hardware (typically the name of the sensor, e.g. "teledyne_rdi_dvl").
    :type hardware_id: string
    """
    def __init__(self, name, hardware_id):
        """
        Constructor.
        """
        # Init vars
        self.set_level_called = False
        self.enabled = False
        self.last_data = 0.0
        self.last_valid_data = 0.0
        self.level = 0
        self.messages = set()
        self.frequency_buffer = []
        self.frequency_buffer_time_limit = 20.0
        self.frequency_buffer_min_data = 5
        self.frequency_buffer_min_time = 2.0
        self.mutex = Lock()

        # Init diagnostic message and publisher
        self.diagnostic_pub = rospy.Publisher(rospy.get_namespace() + "diagnostics", DiagnosticArray, queue_size=40)
        status = DiagnosticStatus()
        status.name = name.split('/')[-1]
        status.hardware_id = hardware_id
        self.diagnostic_msg = DiagnosticArray()
        self.diagnostic_msg.status.append(status)

    def __shrink_frequency_buffer(self, now):
        """
        This method reduces the internal frequency buffer size if needed.

        :param now: Current time.
        :type now: float
        """
        while self.frequency_buffer and now - self.frequency_buffer[0] > self.frequency_buffer_time_limit:
            self.frequency_buffer.pop(0)

    def __compute_frequency(self, now):
        """
        This method uses the frequency buffer to compute the frequency of the reported data.

        :param now: Current time.
        :type now: float
        :return: Returns the frequency. If the computation failed, it returns -1.0.
        :rtype: double
        """
        self.__shrink_frequency_buffer(now)
        if len(self.frequency_buffer) < max(2, self.frequency_buffer_min_data):  # Not enough data
            return -1.0
        if now - self.frequency_buffer[-1] > self.frequency_buffer_min_time:  # Data is getting old
            return -1.0
        time_delta = self.frequency_buffer[-1] - self.frequency_buffer[0]
        if time_delta < max(1e-3, self.frequency_buffer_min_time):  # Not enough data
            return -1.0
        return float(len(self.frequency_buffer) - 1) / time_delta

    def __remove_key_value_impl(self, key):
        """
        This method removes a KeyValue element from the diagnostic message.

        :param key: Key of the KeyValue to be removed.
        :type key: str
        """
        found = False
        i = 0
        while i < len(self.diagnostic_msg.status[0].values) and not found:
            if self.diagnostic_msg.status[0].values[i].key == key:
                found = True
            else:
                i = i + 1
        if found:
            self.diagnostic_msg.status[0].values.remove(self.diagnostic_msg.status[0].values[i])

    def __add_key_value_impl(self, key, value):
        """
        This method adds a KeyValue element to the diagnostic message.

        :param key:  Key of the KeyValue to be added.
        :type key: str
        :param value: Value of the KeyValue to be added.
        :type value: str
        """
        self.__remove_key_value_impl(key)
        self.diagnostic_msg.status[0].values.append(KeyValue(key, value))

    def set_frequency_buffer_time_limit(self, frequency_buffer_time_limit):
        """
        This method is used to set the frequency buffer time limit.

        :param frequency_buffer_time_limit: Frequency buffer time limit.
        :type frequency_buffer_time_limit: float
        """
        self.mutex.acquire()
        self.frequency_buffer_time_limit = frequency_buffer_time_limit
        self.mutex.release()

    def set_frequency_buffer_min_data(self, frequency_buffer_min_data):
        """
        This method is used to set the frequency buffer minimum data amount.

        :param frequency_buffer_min_data: Frequency buffer minimum data amount.
        :type frequency_buffer_min_data: int
        """
        self.mutex.acquire()
        self.frequency_buffer_min_data = frequency_buffer_min_data
        self.mutex.release()

    def set_frequency_buffer_min_time(self, frequency_buffer_min_time):
        """
        This method is used to set the frequency buffer minimum time.

        :param frequency_buffer_min_time: Frequency buffer minimum time.
        :type frequency_buffer_min_time: double
        """
        self.mutex.acquire()
        self.frequency_buffer_min_time = frequency_buffer_min_time
        self.mutex.release()

    def set_enabled(self, enabled):
        """
        This method is used to report whether the node is enabled or disabled.

        :param enabled: True if enabled, false if disabled.
        :type enabled: bool
        """
        self.mutex.acquire()
        self.enabled = enabled
        self.mutex.release()

    def report_data(self, stamp=None):  # When the checksum is ok
        """
        This method is used to report new data.

        :param stamp: ROS time stamp for current time, defaults to None.
        :type stamp: rospy.Time, optional
        """
        self.mutex.acquire()
        if stamp is None:
            self.last_data = rospy.Time.now().to_sec()
        else:
            self.last_data = stamp.to_sec()
        self.frequency_buffer.append(self.last_data)
        self.__shrink_frequency_buffer(self.last_data)
        self.mutex.release()

    def report_valid_data(self, stamp=None):  # When the data can be used
        """
        This method is used to report new valid data.

        :param stamp: ROS time stamp for current time, defaults to None.
        :type stamp: rospy.Time, optional
        """
        self.mutex.acquire()
        if stamp is None:
            self.last_valid_data = rospy.Time.now().to_sec()
        else:
            self.last_valid_data = stamp.to_sec()
        self.last_data = self.last_valid_data
        self.frequency_buffer.append(self.last_valid_data)
        self.__shrink_frequency_buffer(self.last_valid_data)
        self.mutex.release()

    def remove_key_value(self, key):
        """
        This method removes a KeyValue element from the diagnostic message.

        :param key: Key of the KeyValue to be removed.
        :type key: str
        """
        self.mutex.acquire()
        self.__remove_key_value_impl(key)
        self.mutex.release()

    def add_key_value(self, key, value):
        """
        This method adds a KeyValue element to the diagnostic message.

        :param key: Key of the KeyValue to be added.
        :type key: str
        :param value: Value of the KeyValue to be added.
        :type value: str
        """
        self.mutex.acquire()
        if isinstance(value, bool):
            self.__add_key_value_impl(key, str(value).lower())
        else:
            self.__add_key_value_impl(key, str(value))
        self.mutex.release()

    def set_level_and_message(self, level, message=""):
        """
        This method is used to report a new level and to provide an optional message.

        :param level: New diagnostic level.
        :type level: int
        :param message: Message associated with the reported level.
        :type message: str, optional
        """
        self.mutex.acquire()
        if not message:
            if level == DiagnosticStatus.OK:
                self.messages.add("Ok")
            elif level == DiagnosticStatus.WARN:
                self.messages.add("Warn")
            elif level == DiagnosticStatus.ERROR:
                self.messages.add("Error")
            else:
                self.messages.add("Unknown level")
        else:
            self.messages.add(message)
        self.diagnostic_msg.status[0].level = max(self.diagnostic_msg.status[0].level, level)
        self.set_level_called = True
        self.mutex.release()

    def set_message(self, message):
        """
        This method is used to provide a message.

        :param message: Message.
        :type message: str
        """
        self.mutex.acquire()
        self.messages.add(message)
        self.mutex.release()

    def publish(self, stamp=None):
        """
        This method publishes the diagnostics and resets the level and message.

        :param stamp: ROS time stamp for current time, defaults to None.
        :type stamp: rospy.Time, optional
        """
        self.mutex.acquire()
        # Check if a level has been set
        if not self.set_level_called:
            rospy.logwarn("Diagnostic helper publish() is called without setting a level")

        # Header stamp
        if stamp is None:
            self.diagnostic_msg.header.stamp = rospy.Time.now()
        else:
            self.diagnostic_msg.header.stamp = stamp

        # Enabled KeyValue
        self.__add_key_value_impl("enabled", str(self.enabled).lower())

        # Messages
        self.diagnostic_msg.status[0].message = ". ".join(self.messages)

        # Add extra information if enabled
        if self.enabled:
            if self.last_data != 0.0:
                self.__add_key_value_impl("data_age", str(self.diagnostic_msg.header.stamp.to_sec() - self.last_data))
            else:
                self.__remove_key_value_impl("data_age")

            if self.last_valid_data != 0.0:
                self.__add_key_value_impl("valid_data_age", str(self.diagnostic_msg.header.stamp.to_sec() -
                                                                self.last_valid_data))
            else:
                self.__remove_key_value_impl("valid_data_age")

            frequency = self.__compute_frequency(self.diagnostic_msg.header.stamp.to_sec())
            if frequency >= 0.0:
                self.__add_key_value_impl("frequency", str(frequency))
            else:
                self.__remove_key_value_impl("frequency")

        # Publish
        self.diagnostic_pub.publish(self.diagnostic_msg)

        # Reset level and message
        self.diagnostic_msg.status[0].level = 0
        self.messages.clear()
        self.set_level_called = False
        self.mutex.release()
