# Message from GPFPD NMEA String
Header header

string message_id

uint32 week
float64 seconds

float64 yaw
float64 pitch
float64 roll

float64 lat
float64 lon
float64 azimuth

float64 east_velocity
float64 north_velocity
float64 up_velocity

# Baseline length (m)
float32 baseline_length

uint8 num_satellites_ant1
uint8 num_satellites_ant2

string solution_status