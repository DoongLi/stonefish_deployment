cmake_minimum_required(VERSION 2.8.3)
project(cola2_log)

find_package(
  catkin REQUIRED
  COMPONENTS # ROS Dependencies
             rospy
             std_msgs
             std_srvs
             sensor_msgs
             diagnostic_msgs
             rosgraph_msgs
             # COLA2 Dependencies
             cola2_lib_ros
             cola2_msgs)

# System dependencies
find_package(COLA2_LIB REQUIRED)

# Declare things to be passed to dependent projects
catkin_package()

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Shutdown Logger
add_executable(shutdown_logger_startup_process
               startup_processes/shutdown_logger_startup_process.cpp)
target_link_libraries(shutdown_logger_startup_process ${catkin_LIBRARIES}
                      ${COLA2_LIB_LIBRARIES})

# Mission reporter
add_executable(mission_reporter src/mission_reporter_node.cpp)
target_link_libraries(mission_reporter ${catkin_LIBRARIES}
                      ${COLA2_LIB_LIBRARIES} stdc++fs)
