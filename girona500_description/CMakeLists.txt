cmake_minimum_required(VERSION 3.0.2)
project(girona500_description)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  robot_state_publisher

  # COLA2 Dependencies
  cola2_lib_ros
)

# Declare things to be passed to dependent projects
catkin_package()
