# Changelog

## [24.1.0] - 30-01-2024

* Add `teleoperation.launch` to ease installation in controlstations.
* Cleanup unused topics/configs in simulation
* Upgrade cmake minimum version
* Use `--repeat-latched` in rosbag recording
* Change `teleoperation_node` to the new C++ implementation
* Add `cola2_version_node` to know versions of packages from a rosbag
* Add `mission_reporter` node to provide a mission summary for each executed mission
* Add new `captain` parameters for `safety_keep_position_goes_to_ned_origin` and `reset_keep_position_on_navigation_jump`, renamed `controlled_surface_depth` to `safety_keep_position_depth`
* Added more nodes to `diagnostics_aggregator` and `safety_supervisor`
* `navigator` no longer has a `declination_in_degrees` parameter, it is now done inside each IMU/INS driver if required
* Add new `safe_depth_altitude` parameter `min_altitude_starts_at_depth`
* Renamed `last_mission.xml` to `default_mission.xml`

## [20.10.0] - 26-10-2020

* Adapted configuration for new release

## [3.2.0] - 06-11-2019

* Add independent imu launchfile
* Delete node names from configurations
* Rename launchfiles to have common names between vehicles
* Applied cola2 lib refactor changes

## [3.1.0] - 25-02-2019

* First release
