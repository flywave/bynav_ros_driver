#ifndef BYNAV_GPS_DRIVER_PSRDOP_2_H
#define BYNAV_GPS_DRIVER_PSRDOP_2_H

#include <bynav_gps_msgs/Gpdop.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpdopParser : public MessageParser<bynav_gps_msgs::GpdopPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpdopPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::GpdopPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 1163;
  static constexpr size_t ASCII_BODY_FIELDS = 5;
  static constexpr size_t BINARY_BODY_LENGTH = 20;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_PSRDOP_2_H
