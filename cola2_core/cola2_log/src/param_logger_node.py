#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

import os
import rospy
import rosparam
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from cola2_ros.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus
from threading import Lock


class ParamLoggerNode(object):
    """ Log all parameters in rosparam to a topic or to a file """

    def __init__(self):
        """ Constructor """
        # Init node
        rospy.init_node('param_logger')

        # Create mutex
        self.mutex = Lock()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper("param_logger", rospy.get_name())
        self.diagnostic.set_enabled(True)
        rospy.Timer(rospy.Duration(1), self.diagnostics)

        # Publisher and service
        self.pub = rospy.Publisher('~params_string', String, queue_size=1, latch=True)
        self.srv = rospy.Service('~publish_params', Trigger, self.srv_publish)

        # Call it once at the beginning
        self.mutex.acquire()
        self.publish()
        self.mutex.release()

    def diagnostics(self, event):
        """ Publish diagnostics """
        self.diagnostic.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic.publish(event.current_real)

    def publish(self):
        """ Publish all collected parameters """
        # Dump to temp file
        rosparam.dump_params("temp.yaml", "/")

        # Read file into a string message
        msg = String()
        msg.data = open("temp.yaml").read()
        self.pub.publish(msg)

        # Delete temp file
        os.remove("temp.yaml")

    def srv_publish(self, req):
        """ Publish params when service is called """
        #Acquire mutex
        self.mutex.acquire()
        
        # Debug info
        srv = req._connection_header['service']
        who = req._connection_header['callerid']
        rospy.loginfo("service: '{:s}' called from '{:s}'".format(srv, who))

        # Publish
        self.publish()

        #Release mutex
        self.mutex.release()
        return TriggerResponse(True, "parameters published")


if __name__ == '__main__':
    ParamLoggerNode()
    rospy.spin()
