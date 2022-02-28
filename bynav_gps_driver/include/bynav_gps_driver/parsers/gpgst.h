#ifndef BYNAV_GPS_DRIVER_GPGST_H
#define BYNAV_GPS_DRIVER_GPGST_H

#include <bynav_gps_msgs/Gpgst.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpgstParser : public MessageParser<bynav_gps_msgs::GpgstPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpgstPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_GPGST_H
