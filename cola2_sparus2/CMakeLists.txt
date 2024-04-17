cmake_minimum_required(VERSION 3.0.2)
project(cola2_sparus2)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  diagnostic_aggregator
  joy
  rosbridge_server
  rviz
  tf
  topic_tools

  # COLA2 Dependencies
  sparus2_description
)

# Declare things to be passed to dependent projects
catkin_package()

