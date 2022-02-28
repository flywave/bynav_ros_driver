#ifndef BYNAV_GPS_DRIVER_BESTXYZ_H
#define BYNAV_GPS_DRIVER_BESTXYZ_H

#include <bynav_gps_msgs/BynavPJK.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class PtnlPJKParser : public MessageParser<bynav_gps_msgs::BynavPJKPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavPJKPtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_BESTXYZ_H
