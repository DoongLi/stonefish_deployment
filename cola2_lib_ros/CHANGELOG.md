# Changelog

## [24.1.2] - 04-03-2024

* Optionally pass stamp to check if USBL is valid

## [24.1.1] - 04-03-2024

* Add function to check if USBL updates are valid when received from acoustic modem

## [24.1.0] - 30-01-2024

* Added another level of open loop in the setpoint selector
* Add functions to work with invalid navigation messages
* Removed compiler warnings
* Improved documentation
* Added time stamp to request the dynamic transform
* Added conversion from ROS image encodings to OpenCV mat types
* Add missing targets and other files to the install step of the CMake

## [20.10.0] - 26-10-2020

* Added new diagnostic helper class
* Added `setpoint_selector`

## [3.2.0] - 22-10-2019

* Solve bug in `diagnostic_helper.add()` when using `char*`
* Added captain helper to do blocking service calls
* Renamed c++ library to cola2::ros
* Renamed python library to cola2_ros
* Refactor cola 2 lib and splitted into two libraries, one containg ros and another one without ros. This is the one with ros.

## [3.1.0] - 25-02-2019

* First release
