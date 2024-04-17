#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to create captain helper classes.
"""

import rospy
import copy
from threading import Lock
from cola2_msgs.msg import CaptainStatus

class WaitForIdleHelper(object):
    """
    Helper class. Not intended to be used on its own.
    """
    def __init__(self):
        """
        Constructor.
        """
        self.is_idle = False
        self.is_safety = False
        self.mtx = Lock()
        self.first_call = True

    def callback(self, captain_status):
        """
        Callback when received a captain status message.

        :param captain_status: Captain status message.
        :type captain_status: CaptainStatus
        """
        if self.first_call:
            self.first_call = False
            return
        self.mtx.acquire()
        self.is_idle = (captain_status.state == CaptainStatus.IDLE)
        self.is_safety = (captain_status.state == CaptainStatus.SAFETYKEEPPOSITION)
        self.mtx.release()

    def isSafety(self):
        """
        Checks if captain status is in safety keep position.

        :return: True if captain status is in safety keep position. False otherwise.
        :rtype: bool
        """
        self.mtx.acquire()
        is_safety = copy.copy(self.is_safety)
        self.mtx.release()
        return is_safety

    def isIdle(self):
        """
        Checks if captain status is in Idle status.

        :return: True if captain status is Idle. False otherwise.
        :rtype: bool
        """
        self.mtx.acquire()
        is_idle = copy.copy(self.is_idle)
        self.mtx.release()
        return is_idle

def waitForIdle():
    """
    This function blocks until the captain is in Idle state

    :return: True if captain is in idle state. False if its in safety or ros is shutdown.
    :rtype: bool
    """
    helper = WaitForIdleHelper()
    sub = rospy.Subscriber(rospy.get_namespace() + "captain/captain_status", CaptainStatus, helper.callback, queue_size=10)
    while not rospy.is_shutdown():
        if helper.isSafety():
            return False
        if helper.isIdle():
            return True
        rospy.sleep(0.1)
    return False

def callServiceAndWaitForIdle(srv, req, res):
    """
    This function calls a service from the captain, such as a goto, mission... and then blocks
    until the captain is back to idle.

    :param srv: Service client.
    :type srv: ROS Service client
    :param req: Service request.
    :type req: ROS service request
    :param res: Service response.
    :type res: ROS service response
    :return: Returns true if the call and the waiting was succesful, and false otherwise.
    :rtype: bool
    """
    res = srv(req)
    if res.success:
        if waitForIdle():
            return True
        else:
            rospy.logerr("Captain switched to safety state instead of idle")
            return False
    rospy.logerr("Error processing request in callServiceAndWaitForIdle(). Req:\n" +  str(req) + "\nRes:\n" + str(res) + "\n")
    return False
