cmake_minimum_required(VERSION 2.8.3)
project(cola2_control)

add_compile_options(-std=c++11 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/${PROJECT_NAME}/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  cmake_modules
  roscpp
  rosparam
  rospy
  roslib
  tf
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  actionlib
  dynamic_reconfigure

  # COLA2 Dependencies
  cola2_lib_ros
  cola2_msgs
)

# System dependencies
find_package(Boost REQUIRED)
find_package(COLA2_LIB REQUIRED)

# This package has a setup.py. This macro ensures modules and global scripts
# declared therein get installed
catkin_python_setup()

generate_dynamic_reconfigure_options(
        cfg/controller.cfg
)

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controllers low_level_controllers ros_control_base only_thrusters_controller_lib
  # TODO: go and check what is in the headers
  CATKIN_DEPENDS roscpp
                 roslib
                 std_msgs
                 sensor_msgs
                 actionlib
                 cola2_lib_ros
                 cola2_msgs
  DEPENDS Boost COLA2_LIB
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(controllers SHARED
  src/${PROJECT_NAME}/controllers/anchor.cpp
  src/${PROJECT_NAME}/controllers/holonomic_keep_position.cpp
  src/${PROJECT_NAME}/controllers/section.cpp
)
target_link_libraries(controllers ${catkin_LIBRARIES})
add_dependencies(controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(low_level_controllers SHARED
  src/${PROJECT_NAME}/low_level_controllers/auv_controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/merge.cpp
  src/${PROJECT_NAME}/low_level_controllers/ndof_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/pid.cpp
  src/${PROJECT_NAME}/low_level_controllers/poly.cpp
  src/${PROJECT_NAME}/low_level_controllers/request.cpp
)
target_link_libraries(low_level_controllers ${catkin_LIBRARIES})
add_dependencies(low_level_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(ros_control_base SHARED
  src/${PROJECT_NAME}/ros_controller/auv_ros_controller.cpp
)
target_link_libraries(ros_control_base ${catkin_LIBRARIES} low_level_controllers)
add_dependencies(ros_control_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(only_thrusters_controller_lib SHARED
  src/${PROJECT_NAME}/low_level_controllers/only_thrusters_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/only_thruster_allocator.cpp
)
target_link_libraries(only_thrusters_controller_lib ${catkin_LIBRARIES} low_level_controllers)
add_dependencies(only_thrusters_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Add executables
add_executable(keyboard_node src/keyboard_node.cpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(keyboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pilot_node src/pilot_node.cpp)
target_link_libraries(pilot_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} controllers low_level_controllers ${COLA2_LIB_LIBRARIES})
add_dependencies(pilot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(captain_node src/captain_node.cpp)
target_link_libraries(captain_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${COLA2_LIB_LIBRARIES})
add_dependencies(captain_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} controllers low_level_controllers ros_control_base only_thrusters_controller_lib)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(teleoperation_node src/teleoperation_node.cpp)
target_link_libraries(teleoperation_node ${catkin_LIBRARIES} controllers low_level_controllers ros_control_base only_thrusters_controller_lib)
add_dependencies(teleoperation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Mark executable scripts (Python etc.) for installation. In contrast to
# setup.py, you can choose the destination
install(PROGRAMS
  src/${PROJECT_NAME}/joystickbase.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
