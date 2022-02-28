#ifndef BYNAV_GPS_DRIVER_MARK_TIME_H
#define BYNAV_GPS_DRIVER_MARK_TIME_H

#include <bynav_gps_msgs/MarkTime.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class MarkTimeParser : public MessageParser<bynav_gps_msgs::MarkTimePtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::MarkTimePtr
  ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_MARK_TIME_H
