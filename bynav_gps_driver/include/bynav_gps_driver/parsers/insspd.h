#ifndef BYNAV_INSSPD_H
#define BYNAV_INSSPD_H

#include <bynav_gps_msgs/Insspd.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsspdParser : public MessageParser<bynav_gps_msgs::InsspdPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsspdPtr ParseAscii(const BynavSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSSPD_H
