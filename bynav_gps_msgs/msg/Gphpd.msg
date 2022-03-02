# Message from GPHDT NMEA String
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

# Baseline length (m)
float32 baseline_length

uint8 num_satellites_ant1
uint8 num_satellites_ant2

string solution_status