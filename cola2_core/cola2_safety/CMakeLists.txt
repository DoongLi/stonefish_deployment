cmake_minimum_required(VERSION 2.8.3)
project(cola2_safety)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  visualization_msgs

  # COLA2 Dependencies
  cola2_lib_ros
  cola2_msgs
)

# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safety_rules_lib
  CATKIN_DEPENDS rospy
                 std_msgs
                 std_srvs
                 diagnostic_msgs
                 geometry_msgs
                 visualization_msgs
                 cola2_lib_ros
                 cola2_msgs
#  DEPENDS
)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
set(safety_rules_lib_cpp
  src/${PROJECT_NAME}/safety_rules/battery_control_board.cpp
  src/${PROJECT_NAME}/safety_rules/battery_level.cpp
  src/${PROJECT_NAME}/safety_rules/captain.cpp
  src/${PROJECT_NAME}/safety_rules/check_enabled.cpp
  src/${PROJECT_NAME}/safety_rules/combined_water_inside.cpp
  src/${PROJECT_NAME}/safety_rules/common.cpp
  src/${PROJECT_NAME}/safety_rules/comms.cpp
  src/${PROJECT_NAME}/safety_rules/manual.cpp
  src/${PROJECT_NAME}/safety_rules/navigator.cpp
  src/${PROJECT_NAME}/safety_rules/teleoperation.cpp
  src/${PROJECT_NAME}/safety_rules/temperature.cpp
  src/${PROJECT_NAME}/safety_rules/virtual_cage.cpp
  src/${PROJECT_NAME}/safety_rules/watchdog_timer.cpp
  src/${PROJECT_NAME}/safety_rules/water_inside.cpp
)
add_library(safety_rules_lib ${safety_rules_lib_cpp})
target_link_libraries(safety_rules_lib ${catkin_LIBRARIES})
add_dependencies(safety_rules_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add executables
add_executable(safe_depth_altitude_node src/safe_depth_altitude_node.cpp)
target_link_libraries(safe_depth_altitude_node ${catkin_LIBRARIES})
add_dependencies(safe_depth_altitude_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
target_link_libraries(safety_supervisor_node ${catkin_LIBRARIES} safety_rules_lib)
add_dependencies(safety_supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(safety_supervisor_checker_node src/safety_supervisor_checker_node.cpp)
target_link_libraries(safety_supervisor_checker_node ${catkin_LIBRARIES} safety_rules_lib)
add_dependencies(safety_supervisor_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(virtual_cage_node src/virtual_cage_node.cpp)
target_link_libraries(virtual_cage_node ${catkin_LIBRARIES})
add_dependencies(virtual_cage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(watchdog_timer_node src/watchdog_timer_node.cpp)
target_link_libraries(watchdog_timer_node ${catkin_LIBRARIES})
add_dependencies(watchdog_timer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Tests
if(CATKIN_ENABLE_TESTING)
  # Dependencies
  find_package(rostest REQUIRED)
  find_package(COLA2_LIB REQUIRED)

  # Find tools for coverage report
  find_program(GCOV_PATH gcov)
  find_program(LCOV_PATH NAMES lcov lcov.bat lcov.exe lcov.perl)
  find_program(GENHTML_PATH NAMES genhtml genhtml.perl genhtml.bat)
  if(NOT GCOV_PATH)
      message(FATAL_ERROR "gcov not found!")
  endif()
  if(NOT LCOV_PATH)
      message(FATAL_ERROR "lcov not found!")
  endif()
  if(NOT GENHTML_PATH)
      message(FATAL_ERROR "genhtml not found!")
  endif()

  # Set new flags for coverage and testing
  add_compile_options(-std=c++17 -g -O0 --coverage -fprofile-arcs -ftest-coverage -fno-inline -fno-elide-constructors -Wall -Wextra -Wunused -Wswitch-default -Wdiv-by-zero -Wmissing-braces -Wunused-result -Warray-bounds -Wbool-compare -Wbool-operation -Wdangling-else)

  # Create target to perform cleanup
  add_custom_target(${PROJECT_NAME}_coverage_cleanup)
  add_custom_command(TARGET ${PROJECT_NAME}_coverage_cleanup POST_BUILD
    COMMAND ${LCOV_PATH} -d ${PROJECT_BINARY_DIR} -z -q
  )

  # Compile again the library without optimizations and with gcov
  add_library(safety_rules_lib_cov ${safety_rules_lib_cpp})
  target_link_libraries(safety_rules_lib_cov ${catkin_LIBRARIES} gcov)
  add_dependencies(safety_rules_lib_cov ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  # Add tests
  add_rostest_gtest(test_battery_control_board_rule test/test_battery_control_board_rule.launch test/test_battery_control_board_rule.cpp)
  target_link_libraries(test_battery_control_board_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_battery_control_board_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_battery_level_rule test/test_battery_level_rule.launch test/test_battery_level_rule.cpp)
  target_link_libraries(test_battery_level_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_battery_level_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_captain_rule test/test_captain_rule.launch test/test_captain_rule.cpp)
  target_link_libraries(test_captain_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_captain_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_check_enabled_rule test/test_check_enabled_rule.launch test/test_check_enabled_rule.cpp)
  target_link_libraries(test_check_enabled_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_check_enabled_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_combined_water_inside_rule test/test_combined_water_inside_rule.launch test/test_combined_water_inside_rule.cpp)
  target_link_libraries(test_combined_water_inside_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_combined_water_inside_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_common test/test_common.launch test/test_common.cpp)
  target_link_libraries(test_common ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_comms_rule test/test_comms_rule.launch test/test_comms_rule.cpp)
  target_link_libraries(test_comms_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_comms_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_manual_rule test/test_manual_rule.launch test/test_manual_rule.cpp)
  target_link_libraries(test_manual_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_manual_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_navigator_rule test/test_navigator_rule.launch test/test_navigator_rule.cpp)
  target_link_libraries(test_navigator_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_navigator_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_teleoperation_rule test/test_teleoperation_rule.launch test/test_teleoperation_rule.cpp)
  target_link_libraries(test_teleoperation_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_teleoperation_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_temperature_rule test/test_temperature_rule.launch test/test_temperature_rule.cpp)
  target_link_libraries(test_temperature_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_temperature_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_virtual_cage_rule test/test_virtual_cage_rule.launch test/test_virtual_cage_rule.cpp)
  target_link_libraries(test_virtual_cage_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_virtual_cage_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_watchdog_timer_rule test/test_watchdog_timer_rule.launch test/test_watchdog_timer_rule.cpp)
  target_link_libraries(test_watchdog_timer_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_watchdog_timer_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  add_rostest_gtest(test_water_inside_rule test/test_water_inside_rule.launch test/test_water_inside_rule.cpp)
  target_link_libraries(test_water_inside_rule ${catkin_LIBRARIES} safety_rules_lib_cov gcov)
  add_dependencies(test_water_inside_rule ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_coverage_cleanup)

  # Add commands to be run at the end of the tests to generate report
  add_custom_command(TARGET run_tests_${PROJECT_NAME} POST_BUILD
    COMMAND ${LCOV_PATH} -d . -c -o ${PROJECT_NAME}_coverage.info
    COMMAND ${GENHTML_PATH} -o ${PROJECT_NAME}_coverage ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_coverage.info
    COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_NAME}_coverage.info
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMENT "Generating coverage report"
  )
endif()
