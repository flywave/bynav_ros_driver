#ifndef BYNAV_GLOEPHEMERISB_H
#define BYNAV_GLOEPHEMERISB_H

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <vector>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/GnssGloEphemMsg.h>

namespace bynav_gps_driver {

class GloephemerisbParser
    : public MessageParser<bynav_gps_msgs::GnssGloEphemMsgPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GnssGloEphemMsgPtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  static constexpr uint16_t MESSAGE_ID = 723;
  static constexpr size_t BINARY_LENGTH = 144;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver
#endif // BYNAV_GLOEPHEMERISB_H
