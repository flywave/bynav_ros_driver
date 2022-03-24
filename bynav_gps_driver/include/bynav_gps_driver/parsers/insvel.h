#ifndef BYNAV_INSVEL_H
#define BYNAV_INSVEL_H

#include <bynav_gps_msgs/Insvel.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsvelParser : public MessageParser<bynav_gps_msgs::InsvelPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsvelPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::InsvelPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 267;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 40;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSVEL_H
