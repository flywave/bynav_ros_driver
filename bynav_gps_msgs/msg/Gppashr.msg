# Message from GPPASHR NMEA String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

# Heading in degrees (clockwise)
float64 heading

# T: Indicates heading relative to True North
string t

float64 yaw
float64 pitch

float32 undulation

float32 yaw_std
float32 pitch_std
float32 heading_std

string solution_status

uint32 SOLUTION_STATUS_INVALID=0
uint32 SOLUTION_STATUS_SINGLE_POINT=1
uint32 SOLUTION_STATUS_RTK=2
