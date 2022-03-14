#ifndef BYNAV_GPGSV_H
#define BYNAV_GPGSV_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gpgsv.h>

namespace bynav_gps_driver {

class GpgsvParser : MessageParser<bynav_gps_msgs::GpgsvPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpgsvPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPGSV_H
