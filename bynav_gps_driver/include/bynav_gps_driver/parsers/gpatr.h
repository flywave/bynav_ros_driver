#ifndef BYNAV_PSRATR_2_H
#define BYNAV_PSRATR_2_H

#include <bynav_gps_msgs/Gpatr.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpatrParser : public MessageParser<bynav_gps_msgs::GpatrPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpatrPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_PSRATR_2_H
