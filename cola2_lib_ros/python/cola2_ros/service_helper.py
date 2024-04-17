#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to ease managing ros services calls.
"""

import rospy

def connect_to_service(srv_name, srv_type, wait=1.0):
    """
    Connect to the specified service name and type

    :param srv_name: full service name
    :type srv_name: str
    :param srv_type: class of the service to connect to
    :type srv_type: service class
    :param wait: time to wait between tries, defaults to 1.0
    :type wait: float, optional
    :return: connection to the requested service
    :rtype: rospy.ServiceProxy
    """
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(srv_name, wait)
            return rospy.ServiceProxy(srv_name, srv_type)
        except rospy.exceptions.ROSException:
            rospy.logwarn('waiting for client to service: {:s}'.format(srv_name))
            rospy.sleep(wait)
