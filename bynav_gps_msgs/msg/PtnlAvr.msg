# Message from PTNL,AVR NMEA String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

float64 yaw
float64 tilt

# Baseline length (m)
float32 baseline_length

uint32 solution_status
float32 pdop
uint8 num_satellites_used_in_solution

uint32 SOLUTION_STATUS_INVALID=0
uint32 SOLUTION_STATUS_SINGLE_POINT=1
uint32 SOLUTION_STATUS_RTK_FLOATING_AMBIGUITY_SOLUTION=2
uint32 SOLUTION_STATUS_RTK_FIXED_AMBIGUITY_SOLUTION=3
uint32 SOLUTION_STATUS_PSEUDORANGE_DIFFERENTIAL=4
