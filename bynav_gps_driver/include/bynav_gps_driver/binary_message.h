#ifndef BYNAV_GPS_DRIVER_BINARY_MESSAGE_H
#define BYNAV_GPS_DRIVER_BINARY_MESSAGE_H

#include <bynav_gps_driver/binary_header.h>

#include <vector>

namespace bynav_gps_driver {

struct BinaryMessage {
  BinaryHeader header_;
  std::vector<uint8_t> data_;
  uint32_t crc_;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_BINARY_MESSAGE_H
