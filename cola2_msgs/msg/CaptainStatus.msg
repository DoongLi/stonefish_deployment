Header header

# State
uint8 IDLE = 0
uint8 GOTO = 1
uint8 MISSION = 2
uint8 KEEPPOSITION = 3
uint8 SAFETYKEEPPOSITION = 4
uint8 EXTERNALMISSION = 5
uint8 SECTION = 6
uint8 state

# Info message
string message

# List of missions
cola2_msgs/MissionState[] loaded_missions
