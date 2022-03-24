#ifndef BYNAV_RTCM_H
#define BYNAV_RTCM_H

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <vector>

namespace bynav_gps_driver {

class Rtcm {
public:
  const static uint32_t CRC24_TABLE[];
  static constexpr int BUFFER_SIZE = 1024;

  union Rtcm_message_t {
    uint8_t buf[BUFFER_SIZE];
  } in_buffer_;
  uint32_t canary_ = 0xCAFEBABE;

  Rtcm();

  bool ReadCB(uint8_t byte);
  bool ParsingMessage();
  bool NewData();
  volatile bool new_data_;

  void Decode();
  bool CheckCRC();

  size_t message_len_;
  size_t buffer_head_;
  size_t payload_len_;
  bool start_message_;
  bool end_message_;
  uint8_t ck_a_, ck_b_, ck_c_;
  uint16_t msg_type_;
  uint8_t prev_byte_;
  size_t num_errors_;

  enum { START_BYTE = 0xD3, ID_ALL = 0xFFFF };

  typedef enum {
    START,
    GOT_START_FRAME,
    GOT_CLASS,
    GOT_MSG_ID,
    GOT_LENGTH1,
    GOT_LENGTH2,
    GOT_PAYLOAD,
    GOT_CK_A,
    GOT_CK_B,
    GOT_CK_C,
    DONE,
  } parse_state_t;
  parse_state_t parse_state_;

  typedef std::function<void(uint8_t *, size_t, uint16_t, uint32_t)> buffer_cb;
  void RegisterBufferCallback(buffer_cb cb);
  std::vector<buffer_cb> buffer_callbacks_;
};
} // namespace bynav_gps_driver

#endif // BYNAV_RTCM_H