#ifndef BYNAV_GPS_DRIVER_BESTVEL_H_H
#define BYNAV_GPS_DRIVER_BESTVEL_H_H

#include <bynav_gps_driver/parsers/message_parser.h>

#include <bynav_gps_msgs/BynavVelocity.h>

namespace bynav_gps_driver {

class BestvelParser : public MessageParser<bynav_gps_msgs::BynavVelocityPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavVelocityPtr
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::BynavVelocityPtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr uint16_t MESSAGE_ID = 99;
  static constexpr size_t ASCII_LENGTH = 8;
  static constexpr size_t BINARY_LENGTH = 44;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_BESTVEL_H_H
