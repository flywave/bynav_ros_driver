#ifndef BYNAV_INSPOS_H
#define BYNAV_INSPOS_H

#include <bynav_gps_msgs/Inspos.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsposParser : public MessageParser<bynav_gps_msgs::InsposPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsposPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSPOS_H
