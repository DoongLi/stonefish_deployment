#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to ease the task to retreive the namespace of a node.
"""

import rospy


def get_namespace():
    """
    Returns the namespace of the node without double slashes.

    :return: Namespace.
    :rtype: str
    """
    namespace = rospy.get_namespace()
    while namespace.find("//") >= 0:
        namespace = namespace.replace("//", "/")
    if namespace and namespace[-1] == '/':
        namespace = namespace[:-1]
    if not namespace or namespace[0] != '/':
        namespace = '/' + namespace
    return namespace


def get_namespace_no_initial_dash():
    """
    Returns the namespace of the node without double slashes and without the leading dash (useful for frames).

    :return: namespace.
    :rtype: str
    """
    return get_namespace()[1:]


def get_unresolved_node_name():
    """
    Returns the unresolved name of the node (without the namespace).
    If the node name is /swarm1/girona500/node returns node.

    :return: namespace.
    :rtype: str
    """
    return rospy.get_name().split('/')[-1]
