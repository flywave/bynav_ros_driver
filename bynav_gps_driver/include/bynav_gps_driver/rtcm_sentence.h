#ifndef BYNAV_GPS_DRIVER_RTCM_SENTENCE_H
#define BYNAV_GPS_DRIVER_RTCM_SENTENCE_H

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct RtcmSentence {
  uint16_t id;
  std::vector<uint8_t> data;
  uint32_t crc;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_RTCM_SENTENCE_H
