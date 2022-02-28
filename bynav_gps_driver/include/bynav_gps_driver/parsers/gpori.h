#ifndef BYNAV_GPS_DRIVER_GPORI_H
#define BYNAV_GPS_DRIVER_GPORI_H

#include <bynav_gps_msgs/Gpori.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GporiParser : public MessageParser<bynav_gps_msgs::GporiPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GporiPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_GPORI_H
