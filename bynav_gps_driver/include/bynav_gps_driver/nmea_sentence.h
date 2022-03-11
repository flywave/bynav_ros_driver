#ifndef BYNAV_NMEA_SENTENCE_H_
#define BYNAV_NMEA_SENTENCE_H_

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct NmeaSentence {
  std::string id;
  std::vector<std::string> body;
};
} // namespace bynav_gps_driver

#endif // BYNAV_NMEA_SENTENCE_H_
