#ifndef BYNAV_BINARY_HEADER_H_
#define BYNAV_BINARY_HEADER_H_

#include <cstdint>

#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

struct BinaryHeader {
  BinaryHeader()
      : sync0_(0xAA), sync1_(0x44), sync2_(0x12), header_length_(0),
        message_id_(0), message_type_(0), port_address_(0), message_length_(0),
        sequence_(0), idle_time_(0), time_status_(0), week_(0), gps_ms_(0),
        receiver_status_(0), reserved_(0), receiver_sw_version_(0) {}

  uint8_t sync0_;
  uint8_t sync1_;
  uint8_t sync2_;
  uint8_t header_length_;
  uint16_t message_id_;
  int8_t message_type_;
  uint8_t port_address_;
  uint16_t message_length_;
  uint16_t sequence_;
  uint8_t idle_time_;
  uint8_t time_status_;
  uint16_t week_;
  uint32_t gps_ms_;
  uint32_t receiver_status_;
  uint16_t reserved_;
  uint16_t receiver_sw_version_;

  void ParseHeader(const uint8_t *data) {
    sync0_ = data[0];
    sync1_ = data[1];
    sync2_ = data[2];
    header_length_ = data[3];
    message_id_ = ParseUInt16(&data[4]);
    message_type_ = data[6];
    port_address_ = data[7];
    message_length_ = ParseUInt16(&data[8]);
    sequence_ = ParseUInt16(&data[10]);
    idle_time_ = data[12];
    time_status_ = data[13];
    week_ = ParseUInt16(&data[14]);
    gps_ms_ = ParseUInt32(&data[16]);
    receiver_status_ = ParseUInt32(&data[20]);
    reserved_ = ParseUInt16(&data[24]);
    receiver_sw_version_ = ParseUInt16(&data[26]);
  }

  void WriteHeader(uint8_t *data) {
    data[0] = sync0_;
    data[1] = sync1_;
    data[2] = sync2_;
    data[3] = header_length_;
    data[4] = reinterpret_cast<uint8_t *>(&message_id_)[0];
    data[5] = reinterpret_cast<uint8_t *>(&message_id_)[1];
    data[6] = message_type_;
    data[7] = port_address_;
    data[8] = reinterpret_cast<uint8_t *>(&message_length_)[0];
    data[9] = reinterpret_cast<uint8_t *>(&message_length_)[1];
    data[10] = reinterpret_cast<uint8_t *>(&sequence_)[0];
    data[11] = reinterpret_cast<uint8_t *>(&sequence_)[1];
    data[12] = idle_time_;
    data[13] = time_status_;
    data[14] = reinterpret_cast<uint8_t *>(&week_)[0];
    data[15] = reinterpret_cast<uint8_t *>(&week_)[1];
    data[16] = reinterpret_cast<uint8_t *>(&gps_ms_)[0];
    data[17] = reinterpret_cast<uint8_t *>(&gps_ms_)[1];
    data[18] = reinterpret_cast<uint8_t *>(&gps_ms_)[2];
    data[19] = reinterpret_cast<uint8_t *>(&gps_ms_)[3];
    data[20] = reinterpret_cast<uint8_t *>(&receiver_status_)[0];
    data[21] = reinterpret_cast<uint8_t *>(&receiver_status_)[1];
    data[22] = reinterpret_cast<uint8_t *>(&receiver_status_)[2];
    data[23] = reinterpret_cast<uint8_t *>(&receiver_status_)[3];
    data[24] = reinterpret_cast<uint8_t *>(&reserved_)[0];
    data[25] = reinterpret_cast<uint8_t *>(&reserved_)[1];
    data[26] = reinterpret_cast<uint8_t *>(&receiver_sw_version_)[0];
    data[27] = reinterpret_cast<uint8_t *>(&receiver_sw_version_)[1];
  }
};
} // namespace bynav_gps_driver
#endif // BYNAV_BINARY_HEADER_H_
