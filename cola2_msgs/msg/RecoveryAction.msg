# Error level
uint16 NONE = 0
uint16 INFORMATIVE = 1
uint16 ABORT = 2
uint16 ABORT_AND_SURFACE = 3
uint16 EMERGENCY_SURFACE = 4
uint16 DROP_WEIGHT = 5

Header header
uint16 error_level
string error_string
