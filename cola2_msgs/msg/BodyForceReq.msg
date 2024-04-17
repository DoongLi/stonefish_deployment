# Body frame vehicle velocity request.
# No coordinate units, values range +/- 1.
# NB: Coordinate frame used is ROS standard [forward, left, up],
# to simplify integration with arm control systems.

# header.frame_id should be set to the base_link frame of the vehicle.
Header header

GoalDescriptor goal

geometry_msgs/Wrench wrench

# Axes of control to disable, in body frame.
Bool6Axis disable_axis