# Message from GPATR NMEA String

Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds
string position_status

# Baseline length (m)
float32 baseline_length

float32 north_distance
float32 east_distance
float32 up_distance

string orientation_type

float32 yaw
float32 pitch

uint32 ORIENTATION_INVALID=0
uint32 ORIENTATION_SINGLE_POINT=1
uint32 ORIENTATION_PSEUDORANGE_DIFFERENTIAL=2
uint32 ORIENTATION_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 ORIENTATION_RTK_FLOATING_AMBIGUITY_SOLUTION=5
