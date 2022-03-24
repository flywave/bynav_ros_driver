#ifndef BYNAV_GPHPD_H
#define BYNAV_GPHPD_H

#include <bynav_gps_msgs/Gphpd.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GphpdParser : public MessageParser<bynav_gps_msgs::GphpdPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GphpdPtr ParseAscii(const NmeaSentence &sentence) override;

  static constexpr size_t ASCII_BODY_FIELDS = 22;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPHPD_H
