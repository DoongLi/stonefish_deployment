/*    
    This file is a part cola2_stonefish.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 *  cola2_stonefish_node.cpp
 *  cola2_stonefish
 *  
 *  Created by Patryk Cieslak on 09/02/2023.
 *  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
 */

#include <ros/ros.h>
#include <cola2_msgs/DVL.h>
#include <cola2_msgs/NavSts.h>
#include <cola2_msgs/Setpoints.h>
#include <stonefish_ros/DVL.h>
#include <stonefish_ros/INS.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher dvl_pub;
ros::Publisher ins_pub;
ros::Publisher th_pub;
ros::Subscriber dvl_sub;
ros::Subscriber ins_sub;
ros::Subscriber th_sub;

void sfDVLCallback(const stonefish_ros::DVL& in)
{
    cola2_msgs::DVL out;
    out.header = in.header;
    out.velocity = in.velocity;
    out.velocity_covariance = in.velocity_covariance;
    out.altitude = in.altitude;
    out.beams.resize(in.beams.size());
    for(size_t i=0; i<in.beams.size(); ++i)
    {
        out.beams[i].pose = in.beams[i].pose;
        out.beams[i].range = in.beams[i].range;
        out.beams[i].range_covariance = in.beams[i].range_covariance;
        out.beams[i].velocity = in.beams[i].velocity;
        out.beams[i].velocity_covariance = in.beams[i].velocity_covariance;
    }
    dvl_pub.publish(out);
}

void sfINSCallback(const stonefish_ros::INS& in)
{
    cola2_msgs::NavSts out;
    out.header = in.header;
    out.global_position.latitude = in.latitude;
    out.global_position.longitude = in.longitude;
    out.origin.latitude = in.origin_latitude;
    out.origin.longitude = in.origin_longitude;
    out.altitude = in.altitude;
    out.body_velocity.x = in.body_velocity.x;
    out.body_velocity.y = in.body_velocity.y;
    out.body_velocity.z = in.body_velocity.z;
    out.orientation_rate.roll = in.rpy_rate.x;
    out.orientation_rate.pitch = in.rpy_rate.y;
    out.orientation_rate.yaw = in.rpy_rate.z;
    out.position.north = in.pose.north;
    out.position.east = in.pose.east;
    out.position.depth = in.pose.down;
    out.orientation.roll = in.pose.roll;
    out.orientation.pitch = in.pose.pitch;
    out.orientation.yaw = in.pose.yaw;
    out.position_variance.north = in.pose_variance.north;
    out.position_variance.east = in.pose_variance.east;
    out.position_variance.depth = in.pose_variance.down;
    out.orientation_variance.roll = in.pose_variance.roll;
    out.orientation_variance.pitch = in.pose_variance.pitch;
    out.orientation_variance.yaw = in.pose_variance.yaw;
    ins_pub.publish(out);
}

void cola2SetpointsCallback(const cola2_msgs::Setpoints& in)
{
    std_msgs::Float64MultiArray out;
    out.layout.data_offset = 0;
    out.layout.dim.resize(1);
    out.layout.dim[0].label = "setpoints";
    out.layout.dim[0].size = in.setpoints.size();
    out.layout.dim[0].stride = in.setpoints.size();
    out.data = in.setpoints;
    th_pub.publish(out);
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "cola2_stonefish_node");
	ros::NodeHandle nh("~");

    dvl_pub = nh.advertise<cola2_msgs::DVL>("cola2_dvl", 10);
    ins_pub = nh.advertise<cola2_msgs::NavSts>("cola2_navigation", 10);
    th_pub = nh.advertise<std_msgs::Float64MultiArray>("stonefish_thruster_setpoints", 10);

    dvl_sub = nh.subscribe("stonefish_dvl", 10, sfDVLCallback);
    ins_sub = nh.subscribe("stonefish_navigation", 10, sfINSCallback);
    th_sub = nh.subscribe("cola2_thruster_setpoints", 10, cola2SetpointsCallback);
    
    ros::spin();

    return 0;
}