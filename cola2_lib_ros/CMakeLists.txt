cmake_minimum_required(VERSION 2.8.3)
project(cola2_lib_ros)

add_compile_options(-std=c++11 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/cola2_lib_ros/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs

  # COLA2 Dependencies
  cola2_msgs
)

# System dependencies
find_package(COLA2_LIB REQUIRED)
find_package(OpenCV REQUIRED)

# This package has a setup.py. This macro ensures modules and global scripts
# declared therein get installed
catkin_python_setup()

# Declare things to be passed to dependent projects
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cola2_ros cola2_utils
   CATKIN_DEPENDS roscpp
                  rospy
                  tf
                  std_msgs
                  sensor_msgs
                  diagnostic_msgs
                  geometry_msgs
                  nav_msgs
                  cola2_msgs
   DEPENDS OpenCV COLA2_LIB
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${COLA2_LIB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(cola2_ros SHARED
   src/${PROJECT_NAME}/captain_helper.cpp
   src/${PROJECT_NAME}/diagnostic_helper.cpp
   src/${PROJECT_NAME}/navigation_helper.cpp
   src/${PROJECT_NAME}/setpoints_selector.cpp
   src/${PROJECT_NAME}/this_node.cpp
   src/${PROJECT_NAME}/transform_handler.cpp
   src/${PROJECT_NAME}/types.cpp
)
target_link_libraries(cola2_ros ${catkin_LIBRARIES} cola2_utils)
add_dependencies(cola2_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Mark executable scripts (Python etc.) for installation. In contrast to
# setup.py, you can choose the destination
install(PROGRAMS
  python/cola2_ros/captain_helper.py
  python/cola2_ros/diagnostic_helper.py
  python/cola2_ros/navigation_helper.py
  python/cola2_ros/param_loader.py
  python/cola2_ros/service_helper.py
  python/cola2_ros/this_node.py
  python/cola2_ros/transform_handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
install(TARGETS cola2_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES src/cola2_lib_ros/urdf_macros.xacro DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/${PROJECT_NAME})

install(FILES LICENSE.txt README.md CHANGELOG.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})