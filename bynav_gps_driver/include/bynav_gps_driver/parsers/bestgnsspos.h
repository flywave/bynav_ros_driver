#ifndef BYNAV_BESTGNSSPOS_H
#define BYNAV_BESTGNSSPOS_H

#include <bynav_gps_msgs/BynavPosition.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class BestGNSSposParser
    : public MessageParser<bynav_gps_msgs::BynavPositionPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavPositionPtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::BynavPositionPtr
  ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 1492;
  static constexpr size_t BINARY_LENGTH = 72;
  static constexpr size_t ASCII_LENGTH = 21;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_BESTGNSSPOS_H
