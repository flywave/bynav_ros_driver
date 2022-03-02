# INTERFACEMODE COM1 BYNAV BYNAV
InterfaceMode[] modes

# SERIALCONFIG x1 x2
SerialConfig[] serials

# ICOMCONFIG ICOM1 TCP :2000
ICOMConfig[] icom_configs

# FREQUENCYOUT ENABLE 20000000 100000000 POSITIVE 1
FrequencyOut[] freq_outs

# NTRIPCONFIG NCOM1 CLIENT V1 192.168.1.88:8888 NTRIP BYNAV BYNAV ALL
NtripPort[] ntrip_ports

# WORKFREQS L1L2 GPS 
WorkFREQS[] work_freqs

# SET PJKPARA 6378245 298.3 0 0 0 500000 [0.99923 EHT]
bool set_pjk_flag
float64 a
float64 alpha
float64 L0
float64 W0
float64 FN
float64 FE
float64 k0  
string eht

# SETALIGNMENTVEL 5.0
bool set_alignment_vel
float64 alignment_vel

# SETINSROTATION INSRotation X Y Z [XSD YSD ZSD]
bool set_ins_rot_flag
string ins_rot
float64 rot_x
float64 rot_y
float64 rot_z
float64 rot_xsd
float64 rot_ysd
float64 rot_zsd

# SETINSTRANSLATION INSTranslation X Y Z XSD YSD ZSD VEHICLE
bool set_ins_tran_flag
string ins_tran
float64 tran_x
float64 tran_y
float64 tran_z
float64 tran_xsd
float64 tran_ysd
float64 tran_zsd
string tran_input_frame

# INSCALIBRATE RBV NEW
bool set_ins_calibrate_flag
string offset
string trigger
float64 sd_threshold

# DNSCONFIG 1 192.168.1.5
bool set_dns_config_flag
string dns

# IPCONFIG ETHA STATIC 192.168.74.10 255.255.255.0 192.168.74.1
bool set_ip_config_flag
string address_mode
string ip_address
string netmask
string gateway

# SETINSPROFILE LAND/DEFAULT
bool set_ins_profile_flag
string ins_profile

# AUTH ADD E40F99631670CA4F205EB67FE0D2B048
bool set_auth_flag
string auth

# DUALANTENNAPOWER OFF
bool set_dualantenna_power_flag
bool dualantenna_power

# GPSREFWEEK 2553
bool set_gps_ref_week_flag
int32 gps_ref_week

# RTKTYPE ROVER/BASE
bool set_rtk_type_flag
string rtk_type

# ECUTOFF 5
bool set_ecutoff_flag
float32 ecutoff

# SETBASELINE ON 1 0.03
bool set_base_line_flag
float32 base_line_length
float32 base_line_offset

# SNRCUTOFF 40
bool set_snrcutoff_flag
float32 snrcutoff

# SET SHIFTDATUM 0 0 0
bool set_shift_utm_flag
float64 x_offset
float64 y_offset
float64 z_offset

# SET OBSFREQ 2
bool set_obs_freq_flag
int32 obs_freq

# SET FPGARAWFREQ 2
bool set_fpga_raw_freq_flag
int32 fpga_raw_freq

# OUTPUTSOURCE [RAW/KF/INS/ARTK]
bool set_output_source_flag
string output_source

# FIX POSITION 28.234042909 112.888089727 91.0662 
bool set_fix_pos_flag
float64 fix_pos_lon
float64 fix_pos_lat
float64 fix_pos_alt

# RTKTIMEOUT [DIFFAGE]
bool set_rtktimeout_flag
uint32 rtktimeout

# QUALITYCHECK [POS/ORI] [Switch]
bool set_quality_check
string quality_check_pos
bool quality_switch

# HEADINGOFFSET
bool set_heading_offset
float64 heading_offsetin_deg
float64 pitch_offsetin_deg

# VELSMOOTH [PERIOD]
bool set_vel_period
float64 vel_period