#ifndef BYNAV_GPS_DRIVER_BASE_PARSER_H
#define BYNAV_GPS_DRIVER_BASE_PARSER_H

#include <bynav_gps_msgs/BynavMessageHeader.h>

#include <bynav_gps_msgs/BynavExtendedSolutionStatus.h>

#include <bynav_gps_msgs/BynavSignalMask.h>

#include <cstdint>

namespace bynav_gps_driver {
const size_t BYNAV_MESSAGE_HEADER_LENGTH = 10;

const size_t MAX_SOLUTION_STATUS = 22;
const std::string SOLUTION_STATUSES[] = {
    "SOL_COMPUTED", "INSUFFICIENT_OBS",  "NO_CONVERGENCE", "SINGULARITY",
    "COV_TRACE",    "TEST_DIST",         "COLD_START",     "V_H_LIMIT",
    "VARIANCE",     "RESIDUALS",         "RESERVED",       "RESERVED",
    "RESERVED",     "INTEGRITY_WARNING", "RESERVED",       "RESERVED",
    "RESERVED",     "RESERVED",          "PENDING",        "INVALID_FIX",
    "UNAUTHORIZED", "RESERVED",          "INVALID_RATE"};
const size_t MAX_POSITION_TYPE = 80;
const std::string POSITION_TYPES[] = {"NONE",
                                      "FIXEDPOS",
                                      "FIXEDHEIGHT",
                                      "RESERVED",
                                      "FLOATCONV",
                                      "WIDELANE",
                                      "NARROWLANE",
                                      "RESERVED",
                                      "DOPPLER_VELOCITY",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "SINGLE",
                                      "PSRDIFF",
                                      "WAAS",
                                      "PROPOGATED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "L1_FLOAT",
                                      "IONOFREE_FLOAT",
                                      "NARROW_FLOAT",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "L1_INT",
                                      "WIDE_INT",
                                      "NARROW_INT",
                                      "RTK_DIRECT_INS",
                                      "INS_SBAS",
                                      "INS_PSRSP",
                                      "INS_PSRDIFF",
                                      "INS_RTKFLOAT",
                                      "INS_RTKFIXED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "RESERVED",
                                      "PPP_CONVERGING",
                                      "PPP",
                                      "OPERATIONAL",
                                      "WARNING",
                                      "OUT_OF_BOUNDS",
                                      "INS_PPP_CONVERGING",
                                      "INS_PPP",
                                      "UNKNOWN",
                                      "UNKNOWN",
                                      "PPP_BASIC_CONVERGING",
                                      "PPP_BASIC",
                                      "INS_PPP_BASIC",
                                      "INS_PPP_BASIC_CONVERGING"};
const size_t MAX_DATUM = 86;
const std::string DATUMS[] = {
    "BLANK",  "ADIND",  "ARC50", "ARC60",  "AGD66", "AGD84",  "BUKIT",  "ASTRO",
    "CHATM",  "CARTH",  "CAPE",  "DJAKA",  "EGYPT", "ED50",   "ED79",   "GUNSG",
    "GEO49",  "GRB36",  "GUAM",  "HAWAII", "KAUAI", "MAUI",   "OAHU",   "HERAT",
    "HJORS",  "HONGK",  "HUTZU", "INDIA",  "IRE65", "KERTA",  "KANDA",  "LIBER",
    "LUZON",  "MINDA",  "MERCH", "NAHR",   "NAD83", "CANADA", "ALASKA", "NAD27",
    "CARIBB", "MEXICO", "CAMER", "MINNA",  "OMAN",  "PUERTO", "QORNO",  "ROME",
    "CHUA",   "SAM56",  "SAM69", "CAMPO",  "SACOR", "YACAR",  "TANAN",  "TIMBA",
    "TOKYO",  "TRIST",  "VITI",  "WAK60",  "WGS72", "WGS84",  "ZANDE",  "USER",
    "CSRS",   "ADIM",   "ARSM",  "ENW",    "HTN",   "INDB",   "INDI",   "IRL",
    "LUZA",   "LUZB",   "NAHC",  "NASP",   "OGBM",  "OHAA",   "OHAB",   "OHAC",
    "OHAD",   "OHIA",   "OHIB",  "OHIC",   "OHID",  "TIL",    "TOYM"};
const std::string PORT_IDENTIFIERS[] = {
    "NO_PORTS",    "COM1_ALL",     "COM2_ALL",    "COM3_ALL",    "UNUSED",
    "UNUSED",      "THISPORT_ALL", "FILE_ALL",    "ALL_PORTS",   "XCOM1_ALL",
    "XCOM2_ALL",   "UNUSED",       "UNUSED",      "USB1_ALL",    "USB2_ALL",
    "USB3_ALL",    "AUX_ALL",      "XCOM3_ALL",   "UNUSED",      "COM4_ALL",
    "ETH1_ALL",    "IMU_ALL",      "UNUSED",      "ICOM1_ALL",   "ICOM2_ALL",
    "ICOM3_ALL",   "NCOM1_ALL",    "NCOM2_ALL",   "NCOM3_ALL",   "ICOM4_ALL",
    "WCOM1_ALL",   "UNUSED",       "COM1",        "COM1_1",      "COM1_2",
    "COM1_3",      "COM1_4",       "COM1_5",      "COM1_6",      "COM1_7",
    "COM1_8",      "COM1_9",       "COM1_10",     "COM1_11",     "COM1_12",
    "COM1_13",     "COM1_14",      "COM1_15",     "COM1_16",     "COM1_17",
    "COM1_18",     "COM1_19",      "COM1_20",     "COM1_21",     "COM1_22",
    "COM1_23",     "COM1_24",      "COM1_25",     "COM1_26",     "COM1_27",
    "COM1_28",     "COM1_29",      "COM1_30",     "COM1_31",     "COM2",
    "COM2_1",      "COM2_2",       "COM2_3",      "COM2_4",      "COM2_5",
    "COM2_6",      "COM2_7",       "COM2_8",      "COM2_9",      "COM2_10",
    "COM2_11",     "COM2_12",      "COM2_13",     "COM2_14",     "COM2_15",
    "COM2_16",     "COM2_17",      "COM2_18",     "COM2_19",     "COM2_20",
    "COM2_21",     "COM2_22",      "COM2_23",     "COM2_24",     "COM2_25",
    "COM2_26",     "COM2_27",      "COM2_28",     "COM2_29",     "COM2_30",
    "COM2_31",     "COM3",         "COM3_1",      "COM3_2",      "COM3_3",
    "COM3_4",      "COM3_5",       "COM3_6",      "COM3_7",      "COM3_8",
    "COM3_9",      "COM3_10",      "COM3_11",     "COM3_12",     "COM3_13",
    "COM3_14",     "COM3_15",      "COM3_16",     "COM3_17",     "COM3_18",
    "COM3_19",     "COM3_20",      "COM3_21",     "COM3_22",     "COM3_23",
    "COM3_24",     "COM3_25",      "COM3_26",     "COM3_27",     "COM3_28",
    "COM3_29",     "COM3_30",      "COM3_31",     "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "UNUSED",      "UNUSED",       "UNUSED",      "UNUSED",      "UNUSED",
    "SPECIAL",     "SPECIAL_1",    "SPECIAL_2",   "SPECIAL_3",   "SPECIAL_4",
    "SPECIAL_5",   "SPECIAL_6",    "SPECIAL_7",   "SPECIAL_8",   "SPECIAL_9",
    "SPECIAL_10",  "SPECIAL_11",   "SPECIAL_12",  "SPECIAL_13",  "SPECIAL_14",
    "SPECIAL_15",  "SPECIAL_16",   "SPECIAL_17",  "SPECIAL_18",  "SPECIAL_19",
    "SPECIAL_20",  "SPECIAL_21",   "SPECIAL_22",  "SPECIAL_23",  "SPECIAL_24",
    "SPECIAL_25",  "SPECIAL_26",   "SPECIAL_27",  "SPECIAL_28",  "SPECIAL_29",
    "SPECIAL_30",  "SPECIAL_31",   "THISPORT",    "THISPORT_1",  "THISPORT_2",
    "THISPORT_3",  "THISPORT_4",   "THISPORT_5",  "THISPORT_6",  "THISPORT_7",
    "THISPORT_8",  "THISPORT_9",   "THISPORT_10", "THISPORT_11", "THISPORT_12",
    "THISPORT_13", "THISPORT_14",  "THISPORT_15", "THISPORT_16", "THISPORT_17",
    "THISPORT_18", "THISPORT_19",  "THISPORT_20", "THISPORT_21", "THISPORT_22",
    "THISPORT_23", "THISPORT_24",  "THISPORT_25", "THISPORT_26", "THISPORT_27",
    "THISPORT_28", "THISPORT_29",  "THISPORT_30", "THISPORT_31", "FILE",
    "FILE_1",      "FILE_2",       "FILE_3",      "FILE_4",      "FILE_5",
    "FILE_6",      "FILE_7",       "FILE_8",      "FILE_9",      "FILE_10",
    "FILE_11",     "FILE_12",      "FILE_13",     "FILE_14",     "FILE_15",
    "FILE_16",     "FILE_17",      "FILE_18",     "FILE_19",     "FILE_20",
    "FILE_21",     "FILE_22",      "FILE_23",     "FILE_24",     "FILE_25",
    "FILE_26",     "FILE_27",      "FILE_28",     "FILE_29",     "FILE_30",
    "FILE_31"};

double ConvertDmsToDegrees(double dms);

void GetExtendedSolutionStatusMessage(
    uint32_t status, bynav_gps_msgs::BynavExtendedSolutionStatus &msg);

void GetBynavReceiverStatusMessage(
    uint32_t status, bynav_gps_msgs::BynavReceiverStatus &receiver_status_msg);

void GetSignalsUsed(uint32_t mask, bynav_gps_msgs::BynavSignalMask &msg);

double ParseDouble(const uint8_t *buffer);

bool ParseDouble(const std::string &string, double &value);

float ParseFloat(const uint8_t *buffer);

bool ParseFloat(const std::string &string, float &value);

int16_t ParseInt16(const uint8_t *buffer);

bool ParseInt16(const std::string &string, int16_t &value, int32_t base = 10);

int32_t ParseInt32(const uint8_t *buffer);

bool ParseInt32(const std::string &string, int32_t &value, int32_t base = 10);

bool ParseUInt8(const std::string &string, uint8_t &value, int32_t base = 10);

uint16_t ParseUInt16(const uint8_t *buffer);

bool ParseUInt16(const std::string &string, uint16_t &value, int32_t base = 10);

uint32_t ParseUInt32(const uint8_t *buffer);

bool ParseUInt32(const std::string &string, uint32_t &value, int32_t base = 10);

double UtcFloatToSeconds(double utc_float);
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_BASE_PARSER_H
