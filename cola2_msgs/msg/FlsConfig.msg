# This message must be tied to a sensor_msgs/Image message that contains the corresponding polar image. The header 
# timestamp of both messages must be exactly the same.
# The polar image's first row must correspond to the farthest return sample, with the last row being the closest return
# sample.
# The polar image's first column must correspond to the leftmost beam from the POV of the FLS sensor (negative angle 
# value), with the last column being the rightmost beam (positive angle value).

Header 	header

uint32 num_beams
uint32 num_samples

float64 window_start #In meters.
float64 window_end #In meters

float32[] beams_bearings #In radians

float64 speed_of_sound  #In m/s
float64 ping_rate #In Hz
float64 frequency #In Hz



