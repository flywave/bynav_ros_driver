#ifndef BYNAV_PSRDOP_2_H
#define BYNAV_PSRDOP_2_H

#include <bynav_gps_msgs/Gpdop.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpdopParser : public MessageParser<bynav_gps_msgs::GpdopPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpdopPtr ParseAscii(const NmeaSentence &sentence) override;

  static constexpr size_t ASCII_BODY_FIELDS = 7;
  static constexpr uint16_t MESSAGE_ID = 99;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_PSRDOP_2_H
