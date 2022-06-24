# vehicle_ws_state description.

uint64 timestamp		# time since system start (microseconds)

uint8 mission_state		# mission_state of the platform

uint8 WS_VEHICLE_IDLE = 0
uint8 WS_VEHICLE_DIVE = 1
uint8 WS_VEHICLE_WAYPOINTS = 2
uint8 WS_VEHICLE_WAYPOINTS_LOOP = 3
uint8 WS_VEHICLE_NAV = 4
uint8 WS_VEHICLE_LAND = 5

uint32 mission_targets		# Targets for the platform

float32[10] latitude			# Latitude of mission, in degrees
float32[10] longitude			# Longitude of mission, in degrees
