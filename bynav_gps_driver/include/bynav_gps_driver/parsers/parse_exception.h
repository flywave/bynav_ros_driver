#ifndef BYNAV_GPS_DRIVER_PARSE_EXCEPTION_H
#define BYNAV_GPS_DRIVER_PARSE_EXCEPTION_H

#include <exception>

namespace bynav_gps_driver {

class ParseException : public std::runtime_error {
public:
  explicit ParseException(const std::string &error)
      : std::runtime_error(error) {}
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_PARSE_EXCEPTION_H
