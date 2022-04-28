uint64 timestamp			# time since system start (microseconds)
uint8 INDEX_DEFAULT = 0
uint8 INDEX_FORWARD = 1
uint8 INDEX_BACKWARD = 2

uint8 INDEX_SWEPT = 0
uint8 INDEX_RETRACT = 1

uint8[2] swept_actuator_index
uint8[2] retract_actuator_index

float32[2] control

float32 start_timestamp
float32 period
float32 current
