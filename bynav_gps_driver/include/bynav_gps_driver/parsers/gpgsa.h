#ifndef BYNAV_GPS_DRIVER_GPGSA_H
#define BYNAV_GPS_DRIVER_GPGSA_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gpgsa.h>

namespace bynav_gps_driver {

class GpgsaParser : public MessageParser<bynav_gps_msgs::GpgsaPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpgsaPtr
  ParseAscii(const NmeaSentence &sentence) noexcept(false) override;

  static const std::string MESSAGE_NAME;
};
}; // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_GPGSA_H
