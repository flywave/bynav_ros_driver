#ifndef BYNAV_GPS_DRIVER_PTNLAVR_H
#define BYNAV_GPS_DRIVER_PTNLAVR_H

#include <bynav_gps_msgs/PtnlAvr.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class PtnlAvrParser : public MessageParser<bynav_gps_msgs::PtnlAvrPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::PtnlAvrPtr
  ParseAscii(const NmeaSentence &sentence) noexcept(false) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_PTNLAVR_H
