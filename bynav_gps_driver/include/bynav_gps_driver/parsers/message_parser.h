#ifndef BYNAV_MESSAGE_PARSER_H
#define BYNAV_MESSAGE_PARSER_H

#include <bynav_gps_driver/binary_message.h>
#include <bynav_gps_driver/binary_micro_message.h>
#include <bynav_gps_driver/bynav_sentence.h>
#include <bynav_gps_driver/nmea_sentence.h>

#include <bynav_gps_driver/parsers/parse_exception.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

#include <cstdint>

namespace bynav_gps_driver {

template <typename T> class MessageParser {
public:
  virtual ~MessageParser() = default;

  virtual uint32_t GetMessageId() const = 0;

  virtual const std::string GetMessageName() const = 0;

  virtual T ParseBinary(const BinaryMessage &bin_msg) {
    throw ParseException("ParseBinary not implemented.");
  };

  virtual T ParseBinary(const BinaryMicroMessage &bin_msg) {
    throw ParseException("ParseBinary not implemented.");
  };

  virtual T ParseAscii(const BynavSentence &sentence) {
    throw ParseException("ParseAscii not implemented.");
  };

  virtual T ParseAscii(const NmeaSentence &sentence) {
    throw ParseException("ParseAscii not implemented.");
  };
};
} // namespace bynav_gps_driver

#endif // BYNAV_MESSAGE_PARSER_H
