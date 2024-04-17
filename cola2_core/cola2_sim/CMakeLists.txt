cmake_minimum_required(VERSION 2.8.3)
project(cola2_sim)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rospy
  tf
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  dynamic_reconfigure
  gazebo_msgs
  message_generation
  message_runtime

  # COLA2 Dependencies
  cola2_lib_ros
  cola2_msgs
  cola2_control
)

# System dependencies
find_package(Eigen3 REQUIRED)

# TODO: why is this required?
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
               sensor_msgs
               nav_msgs
               geometry_msgs
               gazebo_msgs
               cola2_msgs
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare things to be passed to dependent projects
catkin_package()

add_executable(dynamics_node src/dynamics_node.cpp)
target_link_libraries(dynamics_node ${catkin_LIBRARIES})
add_dependencies(dynamics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sim_internal_sensors_node src/sim_internal_sensors_node.cpp)
target_link_libraries(sim_internal_sensors_node ${catkin_LIBRARIES})
add_dependencies(sim_internal_sensors_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
