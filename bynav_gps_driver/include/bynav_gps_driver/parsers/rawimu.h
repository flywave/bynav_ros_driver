#ifndef BYNAV_RAWIMU_H
#define BYNAV_RAWIMU_H

#include <bynav_gps_msgs/RawIMU.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class RawIMUParser : public MessageParser<bynav_gps_msgs::RawIMUPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::RawIMUPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::RawIMUPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 268;
  static constexpr size_t BINARY_LENGTH = 40;
  static constexpr size_t ASCII_FIELDS = 9;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_RAWIMU_H
