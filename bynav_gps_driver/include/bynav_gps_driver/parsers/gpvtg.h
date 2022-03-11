#ifndef BYNAV_GPVTG_H
#define BYNAV_GPVTG_H

#include <bynav_gps_msgs/Gpvtg.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpvtgParser : public MessageParser<bynav_gps_msgs::GpvtgPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpvtgPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPVTG_H
