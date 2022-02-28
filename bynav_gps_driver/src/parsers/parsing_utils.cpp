#include <swri_string_util/string_util.h>

#include <bynav_gps_driver/parsers/parsing_utils.h>
#include <bynav_gps_msgs/BynavExtendedSolutionStatus.h>
#include <bynav_gps_msgs/BynavSignalMask.h>

namespace bynav_gps_driver {
void GetBynavReceiverStatusMessage(
    uint32_t status, bynav_gps_msgs::BynavReceiverStatus &receiver_status_msg) {
  receiver_status_msg.original_status_code = status;
  receiver_status_msg.error_flag = (status & 0x00000001u) != 0;
  receiver_status_msg.temperature_flag = (status & 0x00000002u) != 0;
  receiver_status_msg.voltage_supply_flag = (status & 0x00000004u) != 0;
  receiver_status_msg.antenna_powered = (status & 0x00000008u) == 0;
  receiver_status_msg.antenna_is_open = (status & 0x00000020u) != 0;
  receiver_status_msg.antenna_is_shorted = (status & 0x00000040u) != 0;
  receiver_status_msg.cpu_overload_flag = (status & 0x00000080u) != 0;
  receiver_status_msg.com1_buffer_overrun = (status & 0x00000100u) != 0;
  receiver_status_msg.com2_buffer_overrun = (status & 0x00000200u) != 0;
  receiver_status_msg.com3_buffer_overrun = (status & 0x00000400u) != 0;
  receiver_status_msg.usb_buffer_overrun = (status & 0x00000800u) != 0;
  receiver_status_msg.rf1_agc_flag = (status & 0x00008000u) != 0;
  receiver_status_msg.rf2_agc_flag = (status & 0x00020000u) != 0;
  receiver_status_msg.almanac_flag = (status & 0x00040000u) != 0;
  receiver_status_msg.position_solution_flag = (status & 0x00080000u) != 0;
  receiver_status_msg.position_fixed_flag = (status & 0x00100000u) != 0;
  receiver_status_msg.clock_steering_status_enabled =
      (status & 0x00200000u) == 0;
  receiver_status_msg.clock_model_flag = (status & 0x00400000u) != 0;
  receiver_status_msg.oemv_external_oscillator_flag =
      (status & 0x00800000u) != 0;
  receiver_status_msg.software_resource_flag = (status & 0x01000000u) != 0;
  receiver_status_msg.aux3_status_event_flag = (status & 0x20000000u) != 0;
  receiver_status_msg.aux2_status_event_flag = (status & 0x40000000u) != 0;
  receiver_status_msg.aux1_status_event_flag = (status & 0x80000000u) != 0;
}

void GetExtendedSolutionStatusMessage(
    uint32_t status, bynav_gps_msgs::BynavExtendedSolutionStatus &msg) {
  msg.original_mask = status;
  msg.advance_rtk_verified = 0x01u & status;
  uint32_t pseudo_iono_correction_mask = (0x0Eu & status) >> 1u;
  switch (pseudo_iono_correction_mask) {
  case 0:
    msg.psuedorange_iono_correction = "Unknown";
    break;
  case 1:
    msg.psuedorange_iono_correction = "Klobuchar Broadcast";
    break;
  case 2:
    msg.psuedorange_iono_correction = "SBAS Broadcast";
    break;
  case 3:
    msg.psuedorange_iono_correction = "Multi-frequency Computed";
    break;
  case 4:
    msg.psuedorange_iono_correction = "PSRDiff Correction";
    break;
  case 5:
    msg.psuedorange_iono_correction = "Bynav Blended Iono Value";
    break;
  default:
    msg.psuedorange_iono_correction = "Unknown";
    break;
  }
}

void GetSignalsUsed(uint32_t mask, bynav_gps_msgs::BynavSignalMask &msg) {
  msg.original_mask = mask;
  msg.gps_L1_used_in_solution = mask & 0x01u;
  msg.gps_L2_used_in_solution = mask & 0x02u;
  msg.gps_L3_used_in_solution = mask & 0x04u;
  msg.glonass_L1_used_in_solution = mask & 0x10u;
  msg.glonass_L2_used_in_solution = mask & 0x20u;
}

double ParseDouble(const uint8_t *buffer) {
  double x;
  std::copy(buffer, buffer + sizeof(double), reinterpret_cast<uint8_t *>(&x));
  return x;
}

bool ParseDouble(const std::string &string, double &value) {
  return swri_string_util::ToDouble(string, value) || string.empty();
}

float ParseFloat(const uint8_t *buffer) {
  float x;
  std::copy(buffer, buffer + sizeof(float), reinterpret_cast<uint8_t *>(&x));
  return x;
}

bool ParseFloat(const std::string &string, float &value) {
  return swri_string_util::ToFloat(string, value) || string.empty();
}

int16_t ParseInt16(const uint8_t *buffer) {
  int16_t number;
  std::copy(buffer, buffer + 2, reinterpret_cast<uint8_t *>(&number));
  return number;
}

bool ParseInt16(const std::string &string, int16_t &value, int32_t base) {
  value = 0;
  if (string.empty()) {
    return true;
  }

  int32_t tmp;
  if (swri_string_util::ToInt32(string, tmp, base) &&
      tmp <= std::numeric_limits<int16_t>::max() &&
      tmp >= std::numeric_limits<int16_t>::min()) {
    value = static_cast<int16_t>(tmp);
    return true;
  }

  return false;
}

int32_t ParseInt32(const uint8_t *buffer) {
  int32_t number;
  std::copy(buffer, buffer + 4, reinterpret_cast<uint8_t *>(&number));
  return number;
}

bool ParseInt32(const std::string &string, int32_t &value, int32_t base) {
  return swri_string_util::ToInt32(string, value, base) || string.empty();
}

uint32_t ParseUInt32(const uint8_t *buffer) {
  uint32_t number;
  std::copy(buffer, buffer + 4, reinterpret_cast<uint8_t *>(&number));
  return number;
}

bool ParseUInt32(const std::string &string, uint32_t &value, int32_t base) {
  return swri_string_util::ToUInt32(string, value, base) || string.empty();
}

bool ParseUInt8(const std::string &string, uint8_t &value, int32_t base) {
  value = 0;
  if (string.empty()) {
    return true;
  }

  uint32_t tmp;
  if (swri_string_util::ToUInt32(string, tmp, base) &&
      tmp <= std::numeric_limits<uint8_t>::max()) {
    value = static_cast<uint8_t>(tmp);
    return true;
  }

  return false;
}

uint16_t ParseUInt16(const uint8_t *buffer) {
  uint16_t number;
  std::copy(buffer, buffer + 2, reinterpret_cast<uint8_t *>(&number));
  return number;
}

bool ParseUInt16(const std::string &string, uint16_t &value, int32_t base) {
  value = 0;
  if (string.empty()) {
    return true;
  }

  uint32_t tmp;
  if (swri_string_util::ToUInt32(string, tmp, base) &&
      tmp <= std::numeric_limits<uint16_t>::max()) {
    value = static_cast<uint16_t>(tmp);
    return true;
  }

  return false;
}

double UtcFloatToSeconds(double utc_float) {
  uint32_t hours = static_cast<uint32_t>(utc_float) / 10000;
  uint32_t minutes = (static_cast<uint32_t>(utc_float) - hours * 10000) / 100;
  double seconds =
      utc_float - static_cast<double>(hours * 10000 + minutes * 100);
  seconds += static_cast<double>(hours * 3600 + minutes * 60);
  return seconds;
}

double ConvertDmsToDegrees(double dms) {
  uint32_t whole_degrees = static_cast<uint32_t>(dms) / 100;
  double minutes = dms - static_cast<double>(whole_degrees * 100);
  double degrees = static_cast<double>(whole_degrees) + minutes / 60.0;
  return degrees;
}
} // namespace bynav_gps_driver
