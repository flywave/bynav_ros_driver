#ifndef BYNAV_INSPVAX_H
#define BYNAV_INSPVAX_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Inspvax.h>

namespace bynav_gps_driver {

class InspvaxParser : public MessageParser<bynav_gps_msgs::InspvaxPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InspvaxPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::InspvaxPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 1465;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 126;
  static constexpr size_t ASCII_FIELDS = 23;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSPVAX_H
