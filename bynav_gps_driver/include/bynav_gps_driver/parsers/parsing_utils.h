#ifndef BYNAV_BASE_PARSER_H
#define BYNAV_BASE_PARSER_H

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

const std::string INS_STATUSES[] = {"INS_INACTIVE",
                                    "INS_ALIGNING",
                                    "INS_HIGH_VARIANCE",
                                    "INS_SOLUTION_GOOD",
                                    "INS_SOLUTION_FREE",
                                    "INS_ALIGNMENT_COMPLETE",
                                    "DETERMINING_ORIENTATION",
                                    "WAITING_INITIALPOS",
                                    "WAITING_AZIMUTH",
                                    "INITIALIZING_BIASES",
                                    "MOTION_DETECT"};

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
                                      "INS_PPP_BASIC_CONVERGING",
                                      "INS_PPP_BASIC"};

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
    "NO_PORTS",  "COM1",  "COM2",  "COM3",  "THISPORT", "FILE",
    "ALL_PORTS", "ETH1",  "IMU",   "ICOM1", "ICOM2",    "ICOM3",
    "ICOM4",     "NCOM1", "NCOM2", "NCOM3", "CCOM1",    "CCOM2",
    "CCOM3",     "MCOM1", "MCOM2", "MCOM3", "MCOM4"};

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

#endif // BYNAV_BASE_PARSER_H
