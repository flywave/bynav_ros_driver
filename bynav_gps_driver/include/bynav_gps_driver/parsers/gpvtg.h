#ifndef BYNAV_GPS_DRIVER_GPVTG_H
#define BYNAV_GPS_DRIVER_GPVTG_H

#include <bynav_gps_msgs/Gpvtg.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpvtgParser : public MessageParser<bynav_gps_msgs::GpvtgPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpvtgPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_GPVTG_H
