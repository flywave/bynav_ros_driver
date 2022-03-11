# Message from GPNTR NMEA String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds
uint32 solution_status

float32 distance

float32 x_distance
float32 y_distance
float32 z_distance

string base_station_id

uint32 SOLUTION_STATUS_INVALID=0
uint32 SOLUTION_STATUS_SINGLE_POINT=1
uint32 SOLUTION_STATUS_PSEUDORANGE_DIFFERENTIAL=2
uint32 SOLUTION_STATUS_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 SOLUTION_STATUS_RTK_FLOATING_AMBIGUITY_SOLUTION=5
