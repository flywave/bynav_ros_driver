#ifndef BYNAV_GPS_DRIVER_INSPVA_H
#define BYNAV_GPS_DRIVER_INSPVA_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Inspva.h>

namespace bynav_gps_driver {

class InspvaParser : public MessageParser<bynav_gps_msgs::InspvaPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InspvaPtr
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::InspvaPtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr uint32_t MESSAGE_ID = 507;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 88;
  static constexpr size_t ASCII_FIELDS = 12;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_INSPVA_H
