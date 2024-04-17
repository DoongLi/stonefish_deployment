# Contains values common to all waypoint request and status messages.
# Waypoint goals with higher priority trump lower priority goals. For
# goals of the same priority, the pilot should follow the latest
# received.

# Name of the requesting node, used with id to identify the request.
string requester

uint32 priority

# PRIORITY DEFINITIONS
uint32 PRIORITY_TELEOPERATION_LOW = 0
uint32 PRIORITY_SAFETY_LOW = 5
uint32 PRIORITY_NORMAL = 10
uint32 PRIORITY_SAFETY = 30
uint32 PRIORITY_TELEOPERATION = 40
uint32 PRIORITY_SAFETY_HIGH  = 50
uint32 PRIORITY_TELEOPERATION_HIGH = 60
