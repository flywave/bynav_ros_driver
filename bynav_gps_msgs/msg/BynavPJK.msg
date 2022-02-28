# Parsed Best GNSS Position 
Header header

BynavMessageHeader bynav_msg_header

string solution_status
string position_type

# Position Data
float64 x
float64 y
float64 z

float32 x_sigma
float32 y_sigma
float32 z_sigma

string velocity_solution_status
string velocity_type

float64 x_vel
float64 y_vel
float64 z_vel

float32 x_vel_sigma
float32 y_vel_sigma
float32 z_vel_sigma

float32 velocity_latency

string base_station_id
float32 diff_age
float32 solution_age
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

BynavExtendedSolutionStatus extended_solution_status

BynavSignalMask signal_mask
