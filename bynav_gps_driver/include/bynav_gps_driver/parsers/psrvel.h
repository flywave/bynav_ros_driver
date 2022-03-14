#ifndef BYNAV_PSRVEL_H_H
#define BYNAV_PSRVEL_H_H

#include <bynav_gps_driver/parsers/message_parser.h>

#include <bynav_gps_msgs/Psrvel.h>

namespace bynav_gps_driver {

class PsrvelParser : public MessageParser<bynav_gps_msgs::PsrvelPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::PsrvelPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::PsrvelPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 100;
  static constexpr size_t ASCII_LENGTH = 8;
  static constexpr size_t BINARY_LENGTH = 44;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_PSRVEL_H_H
