#ifndef BYNAV_GPS_DRIVER_RANGE_H
#define BYNAV_GPS_DRIVER_RANGE_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Range.h>

namespace bynav_gps_driver {
  
class RangeParser : public MessageParser<bynav_gps_msgs::RangePtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::RangePtr
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::RangePtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr size_t BINARY_OBSERVATION_SIZE = 44;
  static constexpr uint16_t MESSAGE_ID = 43;
  static constexpr size_t ASCII_FIELDS = 10;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_RANGE_H
