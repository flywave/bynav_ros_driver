# Message from KSXT String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

# Position Data
float64 lat
float64 lon
float64 height

float32 heading_angle
float32 tilt_angle
float32 velocity_angle
float32 velocity
float32 roll_angle

string position_type
string orientation_type

uint8 num_satellites_used_in_front
uint8 num_satellites_used_in_back

float64 northing
float64 easting
float64 uping

float64 north_velocity
float64 east_velocity
float64 up_velocity

float32 diff_age
uint8 num_satellites_tracked
