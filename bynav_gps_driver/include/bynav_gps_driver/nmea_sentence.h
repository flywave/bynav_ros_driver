#ifndef BYNAV_GPS_DRIVER_NMEA_SENTENCE_H
#define BYNAV_GPS_DRIVER_NMEA_SENTENCE_H

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct NmeaSentence {
  std::string id;
  std::vector<std::string> body;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_NMEA_SENTENCE_H
