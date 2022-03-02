# Message from GPORI NMEA String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds
string position_status

# Baseline length (m)
float32 baseline_length

float32 heading
float32 tilt

float32 baseline_x
float32 baseline_y
float32 baseline_z

uint32 POSITION_STATUS_INVALID=0
uint32 POSITION_STATUS_SINGLE_POINT=1
uint32 POSITION_STATUS_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 POSITION_STATUS_RTK_FLOATING_AMBIGUITY_SOLUTION=5
