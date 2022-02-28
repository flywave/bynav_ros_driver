#ifndef BYNAV_GPS_DRIVER_SHIFTDATUM_H
#define BYNAV_GPS_DRIVER_SHIFTDATUM_H

#include <bynav_gps_msgs/ShiftDatum.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class ShiftDatumParser : public MessageParser<bynav_gps_msgs::ShiftDatumPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::ShiftDatumPtr
  ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_SHIFTDATUM_H
