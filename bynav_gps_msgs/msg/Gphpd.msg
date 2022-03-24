# Message from GPHPD NMEA String
Header header

string message_id

uint32 week
float64 seconds

float64 yaw
float64 pitch
float64 heading

float64 lat
float64 lon
float64 azimuth

float32 east_distance
float32 north_distance
float32 up_distance

float64 east_velocity
float64 north_velocity
float64 up_velocity

float32 east_velocity_diff
float32 north_velocity_diff
float32 up_velocity_diff

# Baseline length (m)
float32 baseline_length

uint8 num_satellites_ant1
uint8 num_satellites_ant2

uint32 solution_status

uint32 SOLUTION_STATUS_INVALID=0
uint32 SOLUTION_STATUS_SINGLE_POINT=1
uint32 SOLUTION_STATUS_PSEUDORANGE_DIFFERENTIAL=2
uint32 SOLUTION_STATUS_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 SOLUTION_STATUS_RTK_FLOATING_AMBIGUITY_SOLUTION=5
