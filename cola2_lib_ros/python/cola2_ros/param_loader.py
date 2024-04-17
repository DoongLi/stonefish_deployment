#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to ease loading ros parameters.
"""

import rospy

def get_ros_params(obj, params):
    """
    Ease param loading from server by assigning them as attributes to the specified object.
    Param names are treated as absoulte if started by /.
    Otherwise they are assumed to be private and namespace and node name are automatically added.

    :param obj: object where to set attributes (self.*).
    :type obj: Object
    :param params: dictionary of {'attribute': ['param_name', default_value]}.
    :type params: dictionary
    :return: all params found in the param server.
    :rtype: bool
    """
    name = rospy.get_name()
    valid = True
    for key in params:
        # Param is in param server
        param_name, param_default = params[key]
        # If not absolute, load with ns and node name
        if param_name[0] != '/':
            param_name = name + '/' + param_name
        if rospy.has_param(param_name):
            # Get from param server
            param_value = rospy.get_param(param_name)
            setattr(obj, key, param_value)
        else:
            # Display message default value loaded
            rospy.logwarn(param_name + ": set to default value = " + str(param_default))
            setattr(obj, key, param_default)
            valid = False
    return valid
