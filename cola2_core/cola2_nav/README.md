# COLA2 NAV

This is a ROS package with nodes to estimate the position of the AUV.

[TOC]

[//]: <> (navigator start)

## navigator

**Node**: /navigator

This node merges data from different navigation sensors of an AUV to estimate the robot position and velocity using an Extended Kalman Filter.

![navigator](doc/readme/navigator.svg)

**Publishers**:

* /diagnostics [[diagnostic_msgs/DiagnosticArray](http://docs.ros.org/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html)]
* /navigator/gps_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
* /navigator/navigation [[cola2_msgs/NavSts](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/NavSts.html)]
* /navigator/odometry [[nav_msgs/Odometry](http://docs.ros.org/noetic/api/nav_msgs/html/msg/Odometry.html)]
* /navigator/usbl_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]

**Subscribers**:

* /navigator/altitude [[sensor_msgs/Range](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Range.html)]
* /navigator/dvl [[cola2_msgs/DVL](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/DVL.html)]
* /navigator/gps [[sensor_msgs/NavSatFix](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/NavSatFix.html)]
* /navigator/imu [[sensor_msgs/Imu](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Imu.html)]
* /navigator/pressure [[sensor_msgs/FluidPressure](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/FluidPressure.html)]
* /navigator/sound_velocity [[cola2_msgs/Float32Stamped](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/Float32Stamped.html)]
* /navigator/usbl [[geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)]

**Services**:

* /navigator/reload_ned [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator/reload_params [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator/reset_navigation [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator/set_depth_sensor_offset [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]

**Parameters**:

* /navigator/depth_sensor_offset
* /navigator/dvl_fallback_delay
* /navigator/dvl_max_v
* /navigator/enable_debug
* /navigator/gps_samples_to_init
* /navigator/initial_state_covariance
* /navigator/initialize_depth_sensor_offset
* /navigator/initialize_filter_from_gps
* /navigator/min_diagnostics_frequency
* /navigator/ned_latitude
* /navigator/ned_longitude
* /navigator/prediction_model_covariance
* /navigator/surface_to_depth_sensor_distance
* /navigator/use_depth_data
* /navigator/use_dvl_data
* /navigator/use_gps_data
* /navigator/use_usbl_data
* /navigator/water_density

[//]: <> (navigator end)

[//]: <> (navigator_position start)

## navigator_position

**Node**: /navigator_position

This node merges data from different navigation sensors of an AUV to estimate the robot position using an Extended Kalman Filter.

![navigator_position](doc/readme/navigator_position.svg)

**Publishers**:

* /diagnostics [[diagnostic_msgs/DiagnosticArray](http://docs.ros.org/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html)]
* /navigator_position/gps_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
* /navigator_position/navigation [[cola2_msgs/NavSts](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/NavSts.html)]
* /navigator_position/odometry [[nav_msgs/Odometry](http://docs.ros.org/noetic/api/nav_msgs/html/msg/Odometry.html)]
* /navigator_position/usbl_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]

**Subscribers**:

* /navigator_position/altitude [[sensor_msgs/Range](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Range.html)]
* /navigator_position/dvl [[cola2_msgs/DVL](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/DVL.html)]
* /navigator_position/gps [[sensor_msgs/NavSatFix](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/NavSatFix.html)]
* /navigator_position/imu [[sensor_msgs/Imu](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Imu.html)]
* /navigator_position/pressure [[sensor_msgs/FluidPressure](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/FluidPressure.html)]
* /navigator_position/sound_velocity [[cola2_msgs/Float32Stamped](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/Float32Stamped.html)]
* /navigator_position/usbl [[geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)]

**Services**:

* /navigator_position/reload_ned [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_position/reload_params [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_position/reset_navigation [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_position/set_depth_sensor_offset [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]

**Parameters**:

* /navigator_position/depth_sensor_offset
* /navigator_position/dvl_fallback_delay
* /navigator_position/dvl_max_v
* /navigator_position/enable_debug
* /navigator_position/gps_samples_to_init
* /navigator_position/initial_state_covariance
* /navigator_position/initialize_depth_sensor_offset
* /navigator_position/initialize_filter_from_gps
* /navigator_position/min_diagnostics_frequency
* /navigator_position/ned_latitude
* /navigator_position/ned_longitude
* /navigator_position/prediction_model_covariance
* /navigator_position/surface_to_depth_sensor_distance
* /navigator_position/use_depth_data
* /navigator_position/use_dvl_data
* /navigator_position/use_gps_data
* /navigator_position/use_usbl_data
* /navigator_position/water_density

[//]: <> (navigator_position end)

[//]: <> (navigator_surface start)

## navigator_surface

**Node**: /navigator_surface

This node merges data from different navigation sensors of an ASV to estimate the robot position and velocity using an Extended Kalman Filter.

![navigator_surface](doc/readme/navigator_surface.svg)

**Publishers**:

* /diagnostics [[diagnostic_msgs/DiagnosticArray](http://docs.ros.org/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html)]
* /navigator_surface/gps_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]
* /navigator_surface/navigation [[cola2_msgs/NavSts](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/NavSts.html)]
* /navigator_surface/odometry [[nav_msgs/Odometry](http://docs.ros.org/noetic/api/nav_msgs/html/msg/Odometry.html)]
* /navigator_surface/usbl_ned [[geometry_msgs/PoseStamped](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/PoseStamped.html)]

**Subscribers**:

* /navigator_surface/altitude [[sensor_msgs/Range](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Range.html)]
* /navigator_surface/dvl [[cola2_msgs/DVL](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/DVL.html)]
* /navigator_surface/gps [[sensor_msgs/NavSatFix](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/NavSatFix.html)]
* /navigator_surface/imu [[sensor_msgs/Imu](http://docs.ros.org/noetic/api/sensor_msgs/html/msg/Imu.html)]
* /navigator_surface/sound_velocity [[cola2_msgs/Float32Stamped](http://api.iquarobotics.com/202401/api/cola2_msgs/html/msg/Float32Stamped.html)]

**Services**:

* /navigator_surface/reload_ned [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_surface/reload_params [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_surface/reset_navigation [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]
* /navigator_surface/set_depth_sensor_offset [[std_srvs/Trigger](http://docs.ros.org/noetic/api/std_srvs/html/srv/Trigger.html)]

**Parameters**:

* /navigator_surface/depth_sensor_offset
* /navigator_surface/dvl_fallback_delay
* /navigator_surface/dvl_max_v
* /navigator_surface/enable_debug
* /navigator_surface/gps_samples_to_init
* /navigator_surface/initial_state_covariance
* /navigator_surface/initialize_depth_sensor_offset
* /navigator_surface/initialize_filter_from_gps
* /navigator_surface/min_diagnostics_frequency
* /navigator_surface/ned_latitude
* /navigator_surface/ned_longitude
* /navigator_surface/prediction_model_covariance
* /navigator_surface/surface_to_depth_sensor_distance
* /navigator_surface/use_depth_data
* /navigator_surface/use_dvl_data
* /navigator_surface/use_gps_data
* /navigator_surface/use_usbl_data
* /navigator_surface/water_density

[//]: <> (navigator_surface end)
