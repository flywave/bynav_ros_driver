#ifndef BYNAV_GPSEPHEMB_H
#define BYNAV_GPSEPHEMB_H

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <vector>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/GnssEphemMsg.h>

namespace bynav_gps_driver {

class GPSEPHEMBParser : public MessageParser<bynav_gps_msgs::GnssEphemMsgPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GnssEphemMsgPtr
  ParseBinary(const BinaryMessage &bin_msg) override;

  static constexpr uint16_t MESSAGE_ID = 7;
};
} // namespace bynav_gps_driver
#endif // BYNAV_GPSEPHEMB_H
