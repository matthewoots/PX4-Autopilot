# vehicle_ws_state description.

uint64 timestamp		# time since system start (microseconds)

bool mission_state		# mission_state of the platform

uint8 WS_VEHICLE_IDLE = 0
uint8 WS_VEHICLE_DIVE = 1
uint8 WS_VEHICLE_WAYPOINTS = 2
uint8 WS_VEHICLE_WAYPOINTS_LOOP = 3
uint8 WS_VEHICLE_LAND = 4

uint32 mission_targets		# Targets for the platform

int32[20] latitude			# Latitude of mission, in 1E-7 degrees
int32[20] longitude			# Longitude of mission, in 1E-7 degrees
