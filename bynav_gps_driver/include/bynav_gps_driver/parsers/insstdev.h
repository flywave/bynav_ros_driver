#ifndef BYNAV_INSSTDEV_H
#define BYNAV_INSSTDEV_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Insstdev.h>

namespace bynav_gps_driver {

class InsstdevParser : public MessageParser<bynav_gps_msgs::InsstdevPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsstdevPtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::InsstdevPtr
  ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 2051;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 52;
  static constexpr size_t ASCII_FIELDS = 14;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSSTDEV_H
