#ifndef BYNAV_GPFPD_H
#define BYNAV_GPFPD_H

#include <bynav_gps_msgs/Gpfpd.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpfpdParser : public MessageParser<bynav_gps_msgs::GpfpdPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpfpdPtr ParseAscii(const NmeaSentence &sentence) override;

  static constexpr size_t ASCII_BODY_FIELDS = 16;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPFPD_H
