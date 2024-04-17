#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


# ROS imports
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.msg import Setpoints
from cola2_msgs.srv import Action


class TestThrusters:
    """Test thrusters from an Action service."""

    def __init__(self):
        """Constructor."""
        rospy.init_node('test_thrusters')

        self.name = rospy.get_name()

        # Create publisher
        self.pub_thrusters_data = rospy.Publisher(
            rospy.get_namespace() + "controller/thruster_setpoints",
            Setpoints, queue_size=1)

        # Create client to disable thrusters service
        rospy.wait_for_service(
            rospy.get_namespace() + 'controller/disable_thrusters', 10)
        try:
            self.disable_thrusters = rospy.ServiceProxy(
                rospy.get_namespace() + 'controller/disable_thrusters', Trigger)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # Create test service
        self.test_srv = rospy.Service('~test',
                                      Action,
                                      self.test)
        rospy.spin()

    def test(self, req):
        """Test service."""
        self.disable_thrusters(TriggerRequest())

        data = Setpoints()
        data.header.frame_id = rospy.get_namespace()[1:] + "safety"
        for p in req.param:
            data.setpoints.append(float(p))

        rospy.loginfo(self.name + ": test thrusters with " +
                      str(data.setpoints) + "\n")
        rate = rospy.Rate(10)
        for i in range(30):
            data.header.stamp = rospy.Time.now()
            self.pub_thrusters_data.publish(data)
            rate.sleep()

        return []


if __name__ == '__main__':
    try:
        TT = TestThrusters()
    except rospy.ROSInterruptException:
        pass
