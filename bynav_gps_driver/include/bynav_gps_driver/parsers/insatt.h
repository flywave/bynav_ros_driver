#ifndef BYNAV_INSATT_H
#define BYNAV_INSATT_H

#include <bynav_gps_msgs/Insatt.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsattParser : public MessageParser<bynav_gps_msgs::InsattPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsattPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::InsattPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 263;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 52;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSATT_H
