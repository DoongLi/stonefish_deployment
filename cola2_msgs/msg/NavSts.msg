# Sent by the navigator

Header 	header

DecimalLatLon global_position
DecimalLatLon origin

# NED position and altitude in metres. 
NED position
float32 altitude

# Body velocities in metres/sec.
geometry_msgs/Point body_velocity

# Orientation and orientation rate are in radians and radians/sec
RPY orientation
RPY orientation_rate

# Variances for position and orientation
NED position_variance
RPY orientation_variance

