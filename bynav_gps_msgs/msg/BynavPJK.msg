# Parsed Best GNSS Position 
Header header

BynavMessageHeader bynav_msg_header

string solution_status
string position_type

# Position Data
float64 x
float64 y

float32 x_sigma
float32 y_sigma
float32 z_sigma

string base_station_id
float32 diff_age
float32 solution_age
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

BynavExtendedSolutionStatus extended_solution_status

BynavSignalMask signal_mask
