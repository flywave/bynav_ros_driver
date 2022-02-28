#ifndef BYNAV_GPS_DRIVER_INSATT_H
#define BYNAV_GPS_DRIVER_INSATT_H

#include <bynav_gps_msgs/Insatt.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsattParser : public MessageParser<bynav_gps_msgs::InsattPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsattPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_INSATT_H
