#!/usr/bin/env python3
# Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module with navigation helper functions.
"""

import rospy
from typing import Optional
from cola2_msgs.msg import NavSts
from geometry_msgs.msg import PoseWithCovarianceStamped


def navigationIsValid(msg: NavSts) -> bool:
    """
    Determine if a navigation message is valid.

    :param msg: Navigation message.
    :type msg: NavSts
    :return: Validity of navigation message.
    :rtype: bool
    """
    if (msg.global_position.latitude == 0.0) and (msg.global_position.longitude == 0.0):
        return False
    return True


def createInvalidNavigation() -> NavSts:
    """
    Create a Invalid Navigation object.

    :return: Invalid navigation.
    :rtype: NavSts
    """
    msg = NavSts()
    msg.header.stamp = rospy.Time.now()
    msg.global_position.latitude = 0.0
    msg.global_position.longitude = 0.0
    return msg


TIME_DELAY_NO_UPDATES: float = 0.99e4  # iquaview uses 1e4 to disable usbls


def usblIsValid(msg: PoseWithCovarianceStamped, stamp: Optional[rospy.Time] = None) -> bool:
    if stamp is None:
        stamp = rospy.Time.now()
    if (stamp - msg.header.stamp).to_sec() > TIME_DELAY_NO_UPDATES:
        return False
    return True
