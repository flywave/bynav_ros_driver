#ifndef BYNAV_TIME_H
#define BYNAV_TIME_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Time.h>

namespace bynav_gps_driver {

class TimeParser : public MessageParser<bynav_gps_msgs::TimePtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::TimePtr
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::TimePtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr size_t BINARY_LENGTH = 44;
  static constexpr uint16_t MESSAGE_ID = 101;
  static constexpr size_t ASCII_FIELD = 11;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_TIME_H
