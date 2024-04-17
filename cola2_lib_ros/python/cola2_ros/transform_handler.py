#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to get transform between different frames.
"""

from __future__ import print_function
import rospy
import tf
from numpy import rad2deg


class TransformHandler(object):
    """
    The TransformHandler class queries and saves transforms with origin at the vehicle frame.
    """
    def __init__(self):
        """
        Constructor that queries the namespace to know vehicle frame."""
        self.transforms = dict()  # transforms from the robot to the sensors
        self.tf_listener = tf.TransformListener()  # transform listener
        self.frame_vehicle = rospy.get_namespace() + "base_link"  # vehicle frame

    def get_transform(self, frame):
        """
        Get a static transform from the map or query the listener and save.

        :param frame: Frame name.
        :type frame: str
        :return: Returns a tuple containig if call was successfull and list with the transform in xyzrpy.
        :rtype: tuple[bool,list[float,float,float],list[float,float,float]]
        """
        # Return identity when frame is vehicle frame
        if frame == self.frame_vehicle:
            return True, [0., 0., 0.], [0., 0., 0.]
        # Look for the transform
        if frame in self.transforms:
            # Found in map
            xyz, rpy = self.transforms[frame]
            return True, xyz, rpy
        else:
            # Need to query it
            ok, xyz, rpy = self.get_dynamic_transform(frame)
            if ok:
                self.transforms[frame] = (xyz, rpy)
                rospy.loginfo("Transform Handler added: {:s}".format(frame))
                rospy.loginfo("trans: {:.3f} {:.3f} {:.3f}".format(*xyz))
                rpy_deg = [rad2deg(v) for v in rpy]
                rospy.loginfo("rpy_deg: {:.3f} {:.3f} {:.3f}".format(*rpy_deg))
                return True, xyz, rpy
        return False, [0., 0., 0.], [0., 0., 0.]

    def get_dynamic_transform(self, frame):
        """
        Get a transform by querying the listener.

        :param frame: Frame name.
        :type frame: str
        :return: Returns a tuple containig if call was successfull and list with the transform in xyzrpy.
        :rtype: tuple[bool,list[float,float,float],list[float,float,float]]
        """
        # Return identity when frame is vehicle frame
        if frame == self.frame_vehicle:
            return True, [0., 0., 0.], [0., 0., 0.]
        # Look for the transform
        try:
            self.tf_listener.waitForTransform(self.frame_vehicle, frame, rospy.Time(0),rospy.Duration(0.5))
            (xyz, quat) = self.tf_listener.lookupTransform(self.frame_vehicle, frame, rospy.Time(0))
            rpy = tf.transformations.euler_from_quaternion(quat)
            return True, xyz, rpy
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal("Unable to get dynamic transform: {:s}".format(frame))
            return False, [0., 0., 0.], [0., 0., 0.]

if __name__ == '__main__':
    # init
    rospy.init_node('transform_handler_test')
    node = TransformHandler()
    print("frame vehicle: {:s}".format(node.frame_vehicle))

    # test
    node.get_transform("base_link")
    node.get_transform("adis_imu")
    node.get_dynamic_transform("adis_imu")
