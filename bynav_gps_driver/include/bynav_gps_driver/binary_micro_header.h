#ifndef BYNAV_BINARY_MICRO_MESSAGE_HEADER_H_
#define BYNAV_BINARY_MICRO_MESSAGE_HEADER_H_

#include <cstdint>

#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

struct BinaryMicroHeader {
  BinaryMicroHeader()
      : sync0_(0xAA), sync1_(0x44), proto_(0x13), header_length_(0),
        message_id_(0), week_(0), gps_ms_(0) {}

  uint8_t sync0_;
  uint8_t sync1_;
  uint8_t proto_;
  uint8_t header_length_;
  uint16_t message_id_;
  uint16_t week_;
  uint32_t gps_ms_;

  void ParseHeader(const uint8_t *data) {
    sync0_ = data[0];
    sync1_ = data[1];
    proto_ = data[2];
    header_length_ = data[3];
    message_id_ = ParseUInt16(&data[4]);
    week_ = ParseUInt16(&data[6]);
    gps_ms_ = ParseUInt32(&data[8]);
  }

  void WriteHeader(uint8_t *data) {
    data[0] = sync0_;
    data[1] = sync1_;
    data[2] = proto_;
    data[3] = header_length_;
    data[4] = reinterpret_cast<uint8_t *>(&message_id_)[0];
    data[5] = reinterpret_cast<uint8_t *>(&message_id_)[1];
    data[14] = reinterpret_cast<uint8_t *>(&week_)[0];
    data[15] = reinterpret_cast<uint8_t *>(&week_)[1];
    data[16] = reinterpret_cast<uint8_t *>(&gps_ms_)[0];
    data[17] = reinterpret_cast<uint8_t *>(&gps_ms_)[1];
    data[18] = reinterpret_cast<uint8_t *>(&gps_ms_)[2];
    data[19] = reinterpret_cast<uint8_t *>(&gps_ms_)[3];
  }
};
} // namespace bynav_gps_driver
#endif // BYNAV_BINARY_MICRO_MESSAGE_HEADER_H_
