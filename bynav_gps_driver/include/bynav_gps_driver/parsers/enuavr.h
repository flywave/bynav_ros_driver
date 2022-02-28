#ifndef BYNAV_GPS_DRIVER_ENUAVR_H
#define BYNAV_GPS_DRIVER_ENUAVR_H

#include <bynav_gps_msgs/BynavEnuAvr.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class EnuavrParser : public MessageParser<bynav_gps_msgs::BynavEnuAvrPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavEnuAvrPtr
  ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_ENUAVR_H
