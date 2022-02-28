#ifndef BYNAV_GPS_DRIVER_HEADER_H
#define BYNAV_GPS_DRIVER_HEADER_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

#include <bynav_gps_msgs/BynavMessageHeader.h>

namespace bynav_gps_driver {

class HeaderParser : public MessageParser<bynav_gps_msgs::BynavMessageHeader> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavMessageHeader
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::BynavMessageHeader
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr uint32_t BINARY_HEADER_LENGTH = 28;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_HEADER_H
