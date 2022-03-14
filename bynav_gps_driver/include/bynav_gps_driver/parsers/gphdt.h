#ifndef BYNAV_GPHDT_H
#define BYNAV_GPHDT_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gphdt.h>

namespace bynav_gps_driver {

class GphdtParser : MessageParser<bynav_gps_msgs::GphdtPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GphdtPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPHDT_H
