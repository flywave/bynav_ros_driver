# Parsed Best GNSS Position 
Header header

BynavMessageHeader bynav_msg_header

string solution_status
string position_type

# Position Data
float64 lat
float64 lon
float64 height

float32 undulation
string datum_id

# Accuracy Statistics (units?)
float32 lat_sigma
float32 lon_sigma
float32 height_sigma
string base_station_id
float32 diff_age
float32 solution_age
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

BynavExtendedSolutionStatus extended_solution_status

BynavSignalMask signal_mask