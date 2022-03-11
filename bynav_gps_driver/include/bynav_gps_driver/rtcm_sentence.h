#ifndef BYNAV_RTCM_SENTENCE_H_
#define BYNAV_RTCM_SENTENCE_H_

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct RtcmSentence {
  uint16_t id;
  std::vector<uint8_t> data;
  uint32_t crc;
};
} // namespace bynav_gps_driver

#endif // BYNAV_RTCM_SENTENCE_H_
