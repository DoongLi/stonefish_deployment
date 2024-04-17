cmake_minimum_required(VERSION 2.8.3)
project(cola2_msgs)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  std_msgs
  sensor_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_runtime
)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
)

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
)

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
               sensor_msgs
               actionlib_msgs
               geometry_msgs
               nav_msgs
)

# Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS std_msgs
                 sensor_msgs
                 actionlib_msgs
                 geometry_msgs
                 nav_msgs
                 message_generation
                 message_runtime
)

install(FILES LICENSE.txt README.md CHANGELOG.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})