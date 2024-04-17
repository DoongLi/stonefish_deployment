cmake_minimum_required(VERSION 2.8.3)
project(cola2_comms)

add_compile_options(-std=c++17 -Wall -Wextra)

# QtCreator add all files
file(GLOB_RECURSE EXTRA_FILES src/*)
file(GLOB_RECURSE EXTRA_INCLUDES include/${PROJECT_NAME}/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES} ${EXTRA_INCLUDES})

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  roscpp
  diagnostic_msgs
  geometry_msgs
  std_msgs
  std_srvs

  # COLA2 Dependencies
  cola2_lib_ros
  cola2_msgs
)

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 diagnostic_msgs
                 geometry_msgs
                 std_msgs
                 std_srvs
                 cola2_lib_ros
                 cola2_msgs
)

# System dependencies
find_package(COLA2_LIB REQUIRED)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executables
add_executable(comms_node src/comms_node.cpp)
target_link_libraries(comms_node ${catkin_LIBRARIES} ${COLA2_LIB_LIBRARIES})
add_dependencies(comms_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
