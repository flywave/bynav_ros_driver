#ifndef BYNAV_HEADING_H
#define BYNAV_HEADING_H

#include <bynav_gps_msgs/Heading.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class HeadingParser : public MessageParser<bynav_gps_msgs::HeadingPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::HeadingPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::HeadingPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 1335;
  static constexpr size_t BINARY_LENGTH = 48;
  static constexpr size_t ASCII_LENGTH = 18;
  static const std::string MESSAGE_NAME;

private:
  uint8_t SolutionSourceToMsgEnum(uint8_t source_mask);
};
} // namespace bynav_gps_driver

#endif // BYNAV_HEADING_H
