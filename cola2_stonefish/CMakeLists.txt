cmake_minimum_required(VERSION 3.10)
project(cola2_stonefish)

find_package(catkin REQUIRED COMPONENTS
  stonefish_ros
  std_msgs
  cola2_msgs
)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS stonefish_ros std_msgs cola2_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cola2_stonefish_node src/cola2_stonefish_node.cpp)
target_link_libraries(cola2_stonefish_node ${catkin_LIBRARIES})
