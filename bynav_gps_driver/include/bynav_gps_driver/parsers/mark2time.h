#ifndef BYNAV_MARK2_TIME_H
#define BYNAV_MARK2_TIME_H

#include <bynav_gps_msgs/MarkTime.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class Mark2TimeParser : public MessageParser<bynav_gps_msgs::MarkTimePtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::MarkTimePtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  bynav_gps_msgs::MarkTimePtr
  ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint32_t MESSAGE_ID = 616;
  static const std::string MESSAGE_NAME;
  static constexpr size_t BINARY_LENGTH = 40;
};
} // namespace bynav_gps_driver

#endif // BYNAV_MARK2_TIME_H
