#!/usr/bin/env python
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

from __future__ import print_function
# Basic ros
import rospy
import tf
# Import msgs
from cola2_msgs.msg import DVL  # dvl
from geometry_msgs.msg import PoseWithCovarianceStamped  # usbl
from sensor_msgs.msg import FluidPressure  # depth
from sensor_msgs.msg import Imu  # imu
from sensor_msgs.msg import NavSatFix  # gps
from sensor_msgs.msg import Range  # altitude
from sensor_msgs.msg import Temperature  # temperature
from diagnostic_msgs.msg import DiagnosticStatus  # diagnostics
from cola2_msgs.msg import Float32Stamped  # sound velocity
from nav_msgs.msg import Odometry  # orientation of the vehicle
# Custom libraries
from cola2.utils.ned import NED
from cola2_ros.diagnostic_helper import DiagnosticHelper
from cola2_ros.transform_handler import TransformHandler
from cola2_ros.param_loader import get_ros_params
from cola2_ros import this_node
# Python libraries
import numpy as np


def transform_from_tf(xyz, rpy):
    """Convert a transform (xyz, rpy) of TransformHandler to numpy (xyz, rotation)."""
    v = np.array([[xyz[0], xyz[1], xyz[2]]]).T
    r = tf.transformations.euler_matrix(rpy[0], rpy[1], rpy[2])[:3, :3]
    # inverse transform
    return -r.T.dot(v), r.T


class SimAUVNavSensors(object):
    """Simulate all the sensors in Girona500."""

    def __init__(self):
        """Constructor that gets config, publishers and subscribers."""
        # Get namespace and config
        self.ns = rospy.get_namespace()
        self.get_config()

        # Initialize other vars
        self.has_odom = False
        self.has_old_odom = False
        self.simulate_altidude = True  # Simulate until altitude received from simulator
        self.ned = NED(self.latitude, self.longitude, 0.0)  # NED frame

        # Set up diagnostics
        self.diagnostic_gps = DiagnosticHelper("gps", rospy.get_name())
        self.diagnostic_gps.set_enabled(True)
        self.diagnostic_depth = DiagnosticHelper("pressure", rospy.get_name())
        self.diagnostic_depth.set_enabled(True)
        self.diagnostic_dvl = DiagnosticHelper("dvl", rospy.get_name())
        self.diagnostic_dvl.set_enabled(True)
        self.diagnostic_imu = DiagnosticHelper("imu", rospy.get_name())
        self.diagnostic_imu.set_enabled(True)

        # Create publishers according to period
        if self.gps_period > 0:
            self.pub_gps = rospy.Publisher(self.ns + 'navigator/gps', NavSatFix, queue_size=2)
        if self.usbl_period > 0:
            self.pub_usbl = rospy.Publisher(self.ns + 'navigator/usbl', PoseWithCovarianceStamped, queue_size=2)
        if self.depth_period > 0:
            self.pub_depth = rospy.Publisher(self.ns + 'navigator/pressure', FluidPressure, queue_size=2)
            self.pub_sound_vel = rospy.Publisher(self.ns + 'navigator/sound_velocity', Float32Stamped, queue_size=2)
            self.pub_temperature = rospy.Publisher(
                self.ns + 'valeport_sound_velocity/temperature', Temperature, queue_size=2)
        if self.dvl_period > 0:
            self.pub_dvl = rospy.Publisher(self.ns + 'navigator/dvl', DVL, queue_size=2)
            self.pub_altitude = rospy.Publisher(self.ns + 'navigator/altitude', Range, queue_size=2)
        if self.imu_period > 0:
            self.pub_imu = rospy.Publisher(self.ns + 'navigator/imu', Imu, queue_size=2)

        # Odometry subscriber
        rospy.Subscriber(self.ns + 'dynamics/odometry', Odometry, self.update_odometry, queue_size=1)

        # Altitude from the simulator
        rospy.Subscriber(self.ns + 'dynamics/altitude', Range, self.update_altitude, queue_size=1)

        # Get transforms
        found = False
        self.tf_handler = TransformHandler()
        while (not found) and (not rospy.is_shutdown()):
            try:
                # GPS
                if self.gps_period > 0:
                    _, gps_xyz, gps_rpy = self.tf_handler.get_transform(self.ns[1:] + 'gps')
                    self.tf_gps = transform_from_tf(gps_xyz, gps_rpy)
                    rospy.loginfo("gps tf loaded")
                # Depth
                if self.depth_period > 0:
                    _, depth_xyz, depth_rpy = self.tf_handler.get_transform(self.ns[1:] + 'pressure')
                    self.tf_depth = transform_from_tf(depth_xyz, depth_rpy)
                    rospy.loginfo("depth tf loaded")
                # DVL
                if self.dvl_period > 0:
                    _, dvl_xyz, dvl_rpy = self.tf_handler.get_transform(self.ns[1:] + 'dvl')
                    # self.tf_dvl = transform_from_tf(dvl_xyz, dvl_rpy)  # not standard inverse
                    v = np.array([[dvl_xyz[0], dvl_xyz[1], dvl_xyz[2]]]).T
                    r = tf.transformations.euler_matrix(dvl_rpy[0], dvl_rpy[1], dvl_rpy[2])[:3, :3]
                    self.tf_dvl = v, r.T  # special case (coordinates from base_link to sensor to transform velocity)
                    rospy.loginfo("dvl tf loaded")
                # IMU
                if self.imu_period > 0:
                    _, imu_xyz, imu_rpy = self.tf_handler.get_transform(self.ns[1:] + 'imu_filter')
                    self.tf_imu_filter = transform_from_tf(imu_xyz, imu_rpy)
                    rospy.loginfo("imu tf loaded")
                # USBL
                if self.usbl_period > 0:
                    _, usbl_xyz, usbl_rpy = self.tf_handler.get_transform(self.ns[1:] + 'modem')
                    self.tf_usbl = transform_from_tf(usbl_xyz, usbl_rpy)
                    rospy.loginfo("usbl tf loaded")
                # All ok
                found = True
            except Exception as e:
                rospy.logwarn("cannot find all transforms")
                rospy.logwarn(e)
                rospy.sleep(2.0)
                # exit(1) This should not exit as sometimes not all tfs are available on the first run

        # Init simulated sensors
        if self.gps_period > 0:
            rospy.Timer(rospy.Duration(self.gps_period), self.publish_gps)
        if self.depth_period > 0:
            rospy.Timer(rospy.Duration(self.depth_period), self.publish_depth)
        if self.dvl_period > 0:
            rospy.Timer(rospy.Duration(self.dvl_period), self.publish_dvl)
        if self.imu_period > 0:
            rospy.Timer(rospy.Duration(self.imu_period), self.publish_imu)
        if self.usbl_period > 0:
            rospy.Timer(rospy.Duration(self.usbl_period), self.publish_usbl)

        # Display message
        rospy.loginfo("initialized")

    def get_config(self):
        """Define and load all necessary parameters from ROS param server."""
        # Define params to load
        param_dict = {
            # absolutes
            'latitude': (self.ns + "navigator/ned_latitude", 41.7777),
            'longitude': (self.ns + "navigator/ned_longitude", 3.0333),
            'water_density': (self.ns + "navigator/water_density", 1030.0),
            # private
            'sea_bottom_depth': ("sea_bottom_depth", 1),
            'sound_speed': ("sound_speed", 1500.0),
            'gps_period': ("gps_period", 0.5),
            'depth_period': ("depth_period", 0.5),
            'dvl_period': ("dvl_period", 0.5),
            'imu_period': ("imu_period", 0.5),
            'usbl_period': ("usbl_period", 5.0),
            # cov
            'gps_position_covariance': ("gps_position_covariance", [0.25, 0.25]),
            'depth_pressure_covariance': ("depth_pressure_covariance", 0.01),
            'dvl_velocity_covariance': ("dvl_velocity_covariance", [0.0015, 0.0015, 0.0015]),
            'imu_orientation_covariance': ("imu_orientation_covariance", [0.0001, 0.0001, 0.0001]),
            'usbl_position_covariance': ("usbl_position_covariance", 0.5),
            # output cov
            'output_gps_position_covariance': ("output_gps_position_covariance", [0.5, 0.5]),
            'output_depth_pressure_covariance': ("output_depth_pressure_covariance", 0.1),
            'output_dvl_velocity_covariance': ("output_dvl_velocity_covariance", [0.02, 0.02, 0.02]),
            'output_imu_orientation_covariance': ("output_imu_orientation_covariance", [0.1, 0.1, 0.1]),
            'output_usbl_position_covariance': ("output_usbl_position_covariance", 3.0),
            # usbl start at depth bigger than
            'usbl_depth_start': ('usbl_depth_start', 4.0)}
        # Load them
        get_ros_params(self, param_dict)

    def update_odometry(self, msg):
        """Get odometry from dynamics to know where the vehicle is."""
        self.odom = msg
        self.rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                             msg.pose.pose.orientation.y,
                                                             msg.pose.pose.orientation.z,
                                                             msg.pose.pose.orientation.w])
        self.has_odom = True

    def update_altitude(self, msg):
        """If altitude is computed outside, stop computing our own."""
        self.simulate_altidude = False
        if (msg.min_range < msg.range < msg.max_range):
            self.altitude = msg.range
        else:
            self.altitude = -1.0              # invalid altitude

    def publish_gps(self, event):
        """Publish GPS according to the current position and if close to surface."""
        # Exit if no odom
        if not self.has_odom:
            rospy.loginfo("waiting for dynamics odometry")
            return
        # Compute position with noise
        north = self.odom.pose.pose.position.x + np.random.normal(0.0, np.sqrt(self.gps_position_covariance[0]))
        east = self.odom.pose.pose.position.y + np.random.normal(0.0, np.sqrt(self.gps_position_covariance[1]))
        ned = np.array([[north, east, 0.0]]).T
        # Transform to sensor
        rot = tf.transformations.euler_matrix(*self.rpy)[:3, :3]
        gps_xyz = rot.dot(self.tf_gps[0])
        ned = ned - np.array([[gps_xyz[0, 0], gps_xyz[1, 0], 0.0]]).T
        # Transform to lat lon
        lat, lon, _ = self.ned.ned2geodetic([ned[0, 0], ned[1, 0], 0.0])
        # Create message
        gps = NavSatFix()
        gps.header.stamp = event.current_real
        gps.header.frame_id = this_node.get_namespace_no_initial_dash() + '/gps'
        gps.status.status = gps.status.STATUS_FIX
        gps.status.service = gps.status.SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = 0.0
        gps.position_covariance[0] = self.output_gps_position_covariance[0]
        gps.position_covariance[4] = self.output_gps_position_covariance[1]
        gps.position_covariance[8] = 1.0
        gps.position_covariance_type = gps.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # Good GPS data only near to surface
        if self.odom.pose.pose.position.z > 1.0:
            gps.status.status = gps.status.STATUS_NO_FIX
        # Publish
        self.pub_gps.publish(gps)
        # Diagnostic message
        self.diagnostic_gps.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic_gps.report_valid_data(event.current_real)
        self.diagnostic_gps.publish(event.current_real)

    def publish_usbl(self, event):
        """Publish USBL according to the current position and if close to surface."""
        # Exit if no odom
        if not self.has_odom:
            rospy.loginfo("waiting for dynamics odometry")
            return
        # Accumulate old message
        if not self.has_old_odom:
            self.old_odom = self.odom
            self.has_old_odom = True
            return
        old = self.old_odom  # message usign old odom
        self.old_odom = self.odom  # save new old odom
        # Exit if not deep enough
        if old.pose.pose.position.z < self.usbl_depth_start:
            return
        # Compute position with noise
        north = old.pose.pose.position.x + np.random.normal(0.0, np.sqrt(self.usbl_position_covariance))
        east = old.pose.pose.position.y + np.random.normal(0.0, np.sqrt(self.usbl_position_covariance))
        ned = np.array([[north, east, 0.0]]).T
        # Transform to sensor
        rot = tf.transformations.euler_matrix(*self.rpy)[:3, :3]
        usbl_xyz = rot.dot(self.tf_usbl[0])
        ned = ned - np.array([[usbl_xyz[0, 0], usbl_xyz[1, 0], 0.0]]).T
        # Transform to lat lon
        lat, lon, _ = self.ned.ned2geodetic([ned[0, 0], ned[1, 0], 0.0])
        # Create message
        usbl = PoseWithCovarianceStamped()
        usbl.header.stamp = event.current_real - rospy.Duration(self.usbl_period)
        usbl.header.frame_id = this_node.get_namespace_no_initial_dash() + '/modem'
        usbl.pose.pose.position.x = lat
        usbl.pose.pose.position.y = lon
        usbl.pose.pose.position.z = 0.0
        usbl.pose.covariance[0] = self.output_usbl_position_covariance
        usbl.pose.covariance[7] = self.output_usbl_position_covariance
        # Publish
        self.pub_usbl.publish(usbl)
        # Diagnostic message
        # self.diagnostic_usbl.set_level_and_message(DiagnosticStatus.OK)

    def publish_depth(self, event):
        """Publish depth according to the current position."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        rot = tf.transformations.euler_matrix(*self.rpy)[:3, :3]
        depth_xyz = rot.dot(self.tf_depth[0])
        depth = self.odom.pose.pose.position.z - depth_xyz[2]
        depth = max(depth, 0.01)
        pressure = depth * self.water_density * 9.80665 + np.random.normal(0.0, np.sqrt(self.depth_pressure_covariance))
        # Pressure
        msg = FluidPressure()
        msg.header.stamp = event.current_real
        msg.header.frame_id = this_node.get_namespace_no_initial_dash() + '/pressure'
        msg.fluid_pressure = pressure
        msg.variance = self.output_depth_pressure_covariance
        self.pub_depth.publish(msg)
        # Sound velocity
        msg = Float32Stamped()
        msg.data = self.sound_speed
        self.pub_sound_vel.publish(msg)
        # Temperature
        msg = Temperature()
        msg.header.stamp = event.current_real
        msg.header.frame_id = this_node.get_namespace_no_initial_dash() + '/pressure'
        msg.temperature = 15.42
        self.pub_temperature.publish(msg)
        # Diagnostic message
        self.diagnostic_depth.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic_depth.report_valid_data(event.current_real)
        self.diagnostic_depth.publish(event.current_real)

    def publish_dvl(self, event):
        """Publish DVL according to the current velocity."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        u = self.odom.twist.twist.linear.x + np.random.normal(0.0, np.sqrt(self.dvl_velocity_covariance[0]))
        v = self.odom.twist.twist.linear.y + np.random.normal(0.0, np.sqrt(self.dvl_velocity_covariance[1]))
        w = self.odom.twist.twist.linear.z + np.random.normal(0.0, np.sqrt(self.dvl_velocity_covariance[2]))
        vel = np.array([u, v, w])
        # Velocity is computed at the gravity center but we want the velocity at the sensor.
        # Vdvl = V + (v.ang.z x dist(dvl->))
        ang_vel = np.array([self.odom.twist.twist.angular.x,
                            self.odom.twist.twist.angular.y, self.odom.twist.twist.angular.z])
        vel = vel + np.cross(ang_vel, self.tf_dvl[0].ravel())
        dvl = self.tf_dvl[1].dot(vel)  # rotate the dvl
        # If simulated altitude, compute it
        if self.simulate_altidude:
            self.altitude = self.sea_bottom_depth - self.odom.pose.pose.position.z
            if self.altitude > 0.5 and self.altitude < 60:
                # Altitude
                msg = Range()
                msg.header.stamp = event.current_real
                msg.header.frame_id = this_node.get_namespace_no_initial_dash() + '/dvl_altitude'
                msg.radiation_type = msg.ULTRASOUND
                msg.field_of_view = 0.2
                msg.min_range = 0.5
                msg.max_range = 80.0
                msg.range = self.altitude
                self.pub_altitude.publish(msg)
        # DVL
        msg = DVL()
        msg.header.stamp = event.current_real
        msg.header.frame_id = this_node.get_namespace_no_initial_dash() + '/dvl'
        msg.velocity.x = dvl[0]
        msg.velocity.y = dvl[1]
        msg.velocity.z = dvl[2]
        msg.velocity_covariance[0] = self.output_dvl_velocity_covariance[0]
        msg.velocity_covariance[4] = self.output_dvl_velocity_covariance[1]
        msg.velocity_covariance[8] = self.output_dvl_velocity_covariance[2]
        msg.altitude = self.altitude
        self.pub_dvl.publish(msg)
        # Diagnostic message
        self.diagnostic_dvl.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic_dvl.report_valid_data(event.current_real)
        self.diagnostic_dvl.publish(event.current_real)

    def publish_imu(self, event):
        """Publish IMU according to the current orientation."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        r = self.rpy[0] + np.random.normal(0.0, np.sqrt(self.imu_orientation_covariance[0]))
        p = self.rpy[1] + np.random.normal(0.0, np.sqrt(self.imu_orientation_covariance[1]))
        y = self.rpy[2] + np.random.normal(0.0, np.sqrt(self.imu_orientation_covariance[2]))
        rot = tf.transformations.euler_matrix(r, p, y)[:3, :3]
        # TODO: Is this right? Maybe vehicle_rpy * self.imu_tf.M instead???
        wrot = np.eye(4)
        # wrot[:3, :3] = self.tf_imu_filter[1].dot(rot)
        wrot[:3, :3] = rot.dot(self.tf_imu_filter[1].T)
        quat = tf.transformations.quaternion_from_matrix(wrot)
        # IMU
        msg = Imu()
        msg.header.stamp = event.current_real
        msg.header.frame_id = this_node.get_namespace_no_initial_dash() + '/imu_filter'
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.orientation_covariance[0] = self.output_imu_orientation_covariance[0]
        msg.orientation_covariance[4] = self.output_imu_orientation_covariance[1]
        msg.orientation_covariance[8] = self.output_imu_orientation_covariance[2]
        # TODO: WARNING! the angular velocity is not rotated!
        msg.angular_velocity = self.odom.twist.twist.angular
        msg.angular_velocity_covariance[0] = 0.1
        msg.angular_velocity_covariance[4] = 0.1
        msg.angular_velocity_covariance[8] = 0.1
        self.pub_imu.publish(msg)
        # Diagnostic message
        self.diagnostic_imu.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic_imu.report_valid_data(event.current_real)
        self.diagnostic_imu.publish(event.current_real)


if __name__ == '__main__':
    # init
    rospy.init_node('sim_auv_nav_sensors')
    node = SimAUVNavSensors()
    rospy.spin()
