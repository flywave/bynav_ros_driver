# Parsed Best GNSS velocity
Header header

BynavMessageHeader bynav_msg_header

string solution_status
string velocity_type

# A measure of the latency in the velocity time tag in seconds.
float32 latency

# Differential age in seconds
float32 age

# Horizontal speed over ground, meters/second
float64 horizontal_speed

# Direction of motion over ground with respect to true North, degrees
float64 track_gnd

# Vertical speed, where positive values correspond to increasing altitude, meters/second
float64 vertical_speed
