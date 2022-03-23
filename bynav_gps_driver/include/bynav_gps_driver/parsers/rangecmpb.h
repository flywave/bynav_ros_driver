#ifndef BYNAV_RANGECMPB_H
#define BYNAV_RANGECMPB_H

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <vector>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/GnssMeasMsg.h>

namespace bynav_gps_driver {

class RangrcmpbParser : public MessageParser<bynav_gps_msgs::GnssMeasMsgPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GnssMeasMsgPtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  static constexpr uint16_t MESSAGE_ID = 140;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver
#endif // BYNAV_RANGECMPB_H
