#ifndef BYNAV_REFSTATION_H
#define BYNAV_REFSTATION_H

#include <bynav_gps_msgs/RefStation.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class RefStationParser : public MessageParser<bynav_gps_msgs::RefStationPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::RefStationPtr
  ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_REFSTATION_H
