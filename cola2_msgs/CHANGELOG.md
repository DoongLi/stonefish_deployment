# Changelog

## [24.1.0] - 30-01-2024

* Added generic FLS configuration message: FLSConfig.msg
* Added generic image acquisition settings message: ImageAcquisitionSettings
* Added key value array message: KeyValueArray
* Added generic sidescan sonar configuration message: SSSConfig
* Added file to installation step of cmake

## [20.10.0] - 26-10-2020

* Updated pilot actionlib, and also the goto and section services
* Updated recovery action message to allow for drop weight

## [3.2.0] - 22-10-2019

* Change `SafetySupervisorStatus.status_code` from `int32` to `uint32`
* Add `CaptainStateFeedback` message
* Add `KeyValue` message
* Add `Section` service
* Delete `MissionStatus` message
* Modified the CaptainStatus message to incorporate a list of the new MissionState message (used for captain loaded missions)

## [3.1.0] - 25-02-2019

* First release
