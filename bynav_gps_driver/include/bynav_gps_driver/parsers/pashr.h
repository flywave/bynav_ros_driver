#ifndef BYNAV_GPPASHR_H
#define BYNAV_GPPASHR_H

#include <bynav_gps_msgs/Pashr.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class PashrParser : public MessageParser<bynav_gps_msgs::PashrPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::PashrPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPPASHR_H
