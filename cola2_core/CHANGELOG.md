# Changelog

## [24.1.4] - 11-03-2024

* `cola2_control`: improved controller.yaml so that thruster setpoints are allowed to reach unity

## [24.1.3] - 04-03-2024

* `cola2_control`: solve problem with test_thrusters node
* `cola2_nav`: discard usbls that IQUAview sends as invalid

## [24.1.2] - 21-02-2024

* `cola2_log`: fixed bug to handle generic errors during repo processing for version

## [24.1.1] - 20-02-2024

* `cola2_log`: fixed bug to handle repos without origin

## [24.1.0] - 30-01-2024

* `cola2_control`: teleoperation node reimplemented in C++
* `cola2_control`: disable thrusters now resets emergency ramp
* `cola2_control`: added automatic open loop teleoperation when navigation fails
* `cola2_control`: controller now sends zeros when the thrusters are disabled
* `cola2_nav`: added valid navigation checks
* `cola2_safety`: comms rule now also checks modem
* `cola2_log`: log versions of packages in bagfile, added mission report generator
* `cola2_sim`: solved IMU TF issue in sim_auv_nav_sensors node
* `cola2_safety`: added min altitude starts at depth parameter
* `cola2_control`: added parameter to reset keep position on navigation jump
* `cola2_control`: added parameter to send the robot to NED origin on safety keep position
* `cola2_control`: solved issue in section controller that causes the vehicle to turn when moving vertically
* `cola2_control`: captain now publishes mission as latched string on enable
* `cola2_control`: now pose controller td is used
* `cola2_control`: last mission is now default mission
* `cola2_control`: pitch is no longer disabled
* `cola2_control`: solved PID derivative issue
* `cola2_control`: better termios settings and rosout in keyboard node
* `cola2_control`: change param name from `controlled_surface_depth` to `safety_keep_position_depth`
* `cola2_nav`: added ROS warnings when filter update fails
* `cola2_nav`: added position navigator
* `cola2_nav`: declination is not applied anymore (imu should do it)
* `cola2_comms`: add service `reset_recovery_action` to disable aborts via wifi once on surface

## [20.10.10] - 08-02-2023

* `cola2_control`: add navigation checks in captain

## [20.10.9] - 08-07-2022

* `cola2_log`: correct bug in default param handler when using python3 (ROS noetic)

## [20.10.8] - 30-03-2022

* `cola2_control`: solve issue in reset keep position

## [20.10.7] - 15-12-2021

* `cola2_control`: solve issue in vertical velocity when using vertical pose controller

## [20.10.6] - 14-04-2021

* `cola2_control`: solve issue in pilot section min surge velocity

## [20.10.5] - 07-04-2021

* `cola2_control`: improve teleoperation behavior when ack is lost

## [20.10.4] - 22-03-2021

* `cola2_safety`: solve safety issue with ros::Time zero
* `cola2_control`: solve issue with ros::Time zero in pilot and captain nodes
* `cola2_nav`: solve issue with ros::Time zero in navigator node
* `cola2_control`: solve compiler warnings in Ubuntu 20.04

## [20.10.3] - 04-03-2021

* `cola2_safety`: increased GPS surface timeout in navigator safety rule
* `cola2_safety`: solved issues in unit tests

## [20.10.2] - 11-01-2021

* `cola2_control`: solve Python compatibility issues with Ubuntu 20.04.
* `cola2_log`: solve Python compatibility issues with Ubuntu 20.04.

## [20.10.1] - 07-01-2021

* `cola2_nav`: forget usbl position history when reloading parameters or changing NED origin.

## [20.10.0] - 26-10-2020

* `cola2_log`: Added shutdown logger
* `cola2_control`: Pilot and captain simplification. Now using a single actionlib. The goto and section services have also been updated.
* `cola2_nav`: Keep full pose history instead of only positions `xy`.
* `cola2_nav`: Ease inheritance to implement custom navigators on top of the one available.
* Improved general diagnostics using the new diagnostic helper class
* `cola2_safety`: New package using C++, and without vehicle status.
* `cola2_control`: Solved issue with disable mission with no mission name when not in mission state.
* `cola2_control`: Added first version of open loop node.
* `cola2_sim`: The dynamics node now uses the new setpoints selector from cola2_lib_ros.
* `cola2_control`: New set zero velocity in the controller.

## [3.2.5] - 28-08-2020

* `cola2_nav`: Solved negative pressure check issue in pressure sensor callback.

## [3.2.4] - 18-03-2020

* `cola2_control`: Corrected issue in section services.

## [3.2.3] - 30-01-2020

* `cola2_nav`: Corrected magnetic declination sign so the magnetic declination is properly taken into account into the navigation filter. Magnetic declinations from websites such as [http://www.magnetic-declination.com/](http://www.magnetic-declination.com/) can be now used with the sign given there.

## [3.2.2] - 29-01-2020

* `cola2_safety`: Solved issue in disable mission service of recovery actions node.

## [3.2.1] - 10-01-2020

* `cola2_nav`: Added missing dependency to be able to compile without installing cola2_lib in the system (related to cola2_lib hotfix 3.2.2).

## [3.2.0] - 22-10-2019

* Add `_node` to all nodes source code files
* Add tags to `README.md` for auto-documentation
* Apply `cola2_lib` refactor changes
* Change `std_srvs/Empty` services to `std_srvs/Trigger`
* Delete node name from example configurations
* Keep nodes alive for auto-documentation
* `cola2_comms`: new acoustic command to reset vehicle timeout
* `cola2_control`: added mission pause and resume services
* `cola2_control`: added state_feedback topic to publish information about on-going state for all services
* `cola2_control`: all captain services are nonblocking now
* `cola2_control`: captain service calls now have a timeout of 5 seconds to ensure captain is not blocked
* `cola2_control`: captain_node is now single-threaded
* `cola2_control`: changed captain_status topic to reflect the internal state of the captain and the list of loaded missions
* `cola2_control`: enable/disable mission services can enable/disable a mission by name
* `cola2_control`: improve checks on mission validity before starting the mission
* `cola2_control`: keyboard_node is now single-threaded
* `cola2_control`: removed default mission nonblocking service from captain
* `cola2_log`: adapt default_param_handler to take filename as node name
* `cola2_log`: default_param_handler service to update single parameter
* `cola2_nav`: add service to only reload NED info
* `cola2_nav`: delete navigator/altitude_filtered topic (altitude output of navigator now only in navigator/navigation)
* `cola2_nav`: solve bug in too old USBL updates
* `cola2_sim`: dynamics, solve bug in negative thruster setpoints

## [3.1.1] - 15-04-2019

* Solved bug when loading mission actions with parameters

## [3.1.0] - 25-02-2019

* First release

