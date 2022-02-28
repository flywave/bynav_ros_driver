#ifndef BYNAV_GPS_DRIVER_BYNAV_SENTENCE_H
#define BYNAV_GPS_DRIVER_BYNAV_SENTENCE_H

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct BynavSentence {
  std::string id;
  std::vector<std::string> header;
  std::vector<std::string> body;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_BYNAV_SENTENCE_H
