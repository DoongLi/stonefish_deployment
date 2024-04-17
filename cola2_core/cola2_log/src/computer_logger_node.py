#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

# ROS imports
import rospy
import subprocess
import sys
import psutil
from cola2_ros.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32


class ComputerLogger(object):
    """ Publishes CPU and RAM usage data and temperature from the vehicle computer """

    def __init__(self, name):
        """ Constructor """
        # Init class vars
        self.name = name

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper("computer_logger", self.name)
        self.diagnostic.set_enabled(True)

        # Publisher
        self.pub_temp = rospy.Publisher("~temperature", Temperature, queue_size = 2)
        self.pub_ram = rospy.Publisher("~ram_usage", Float32, queue_size = 2)
        self.pub_cpu = rospy.Publisher("~cpu_usage", Float32, queue_size = 2)

        # Start timer
        rospy.Timer(rospy.Duration(1), self.iterate)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

    def iterate(self, event):
        """ Callback from the main timer """
        try:
            # Execute sensors command and parse output
            p = subprocess.Popen(['sensors'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            out, err = p.communicate()
            splitted = out.decode("utf-8").split("\n")

            max_core_temp = -99

            # Get max core temperature
            for line in splitted:
                if "Core" in line:
                    end = line.index("(") - 6
                    start = end - 6
                    temp = float(line[start:end])
                    if temp > max_core_temp:
                        max_core_temp = temp

            # Use psutil to get cpu and ram usage
            cpu_usage = psutil.cpu_percent()
            ram_usage = psutil.virtual_memory().percent

            self.diagnostic.add_key_value("cpu_usage", cpu_usage)
            self.diagnostic.add_key_value("ram_usage", ram_usage)
            self.diagnostic.add_key_value("temperature", max_core_temp)
            self.diagnostic.set_level_and_message(DiagnosticStatus.OK)
            self.diagnostic.report_valid_data(event.current_real)
            self.diagnostic.publish(event.current_real)

            msg = Temperature()
            msg.header.stamp = rospy.Time.now()
            msg.temperature = max_core_temp
            msg.variance = 0  # Unknown
            self.pub_temp.publish(msg)

            msg = Float32()
            msg.data = cpu_usage
            self.pub_cpu.publish(msg)

            msg.data = ram_usage
            self.pub_ram.publish(msg)

        except:
            rospy.logwarn("%s: unable to get data: %s", self.name, sys.exc_info()[0])


if __name__ == '__main__':
    try:
        rospy.init_node('computer_logger')
        computer_logger = ComputerLogger(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
