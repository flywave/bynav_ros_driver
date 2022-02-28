#ifndef BYNAV_GPS_DRIVER_INSVEL_H
#define BYNAV_GPS_DRIVER_INSVEL_H

#include <bynav_gps_msgs/Insvel.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsvelParser : public MessageParser<bynav_gps_msgs::InsvelPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsvelPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_INSVEL_H
