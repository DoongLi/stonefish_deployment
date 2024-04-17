Header header

# State
uint8 ACTIVE = 0
uint8 SUCCESS = 1
uint8 FAILURE = 2
uint8 STOPPED = 3
uint8 state

# An estimate of the remaining time
float64 time_to_finish

# Name of the mode
string name

# Always increasing id assigned by the captain
uint16 id

# Extra information as a key-value list
cola2_msgs/KeyValue[] keyvalues
