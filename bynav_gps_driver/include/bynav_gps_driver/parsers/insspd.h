#ifndef BYNAV_INSSPD_H
#define BYNAV_INSSPD_H

#include <bynav_gps_msgs/Insspd.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class InsspdParser : public MessageParser<bynav_gps_msgs::InsspdPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::InsspdPtr ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::InsspdPtr ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 266;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 40;
};
} // namespace bynav_gps_driver

#endif // BYNAV_INSSPD_H
