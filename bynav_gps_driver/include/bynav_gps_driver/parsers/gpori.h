#ifndef BYNAV_GPORI_H
#define BYNAV_GPORI_H

#include <bynav_gps_msgs/Gpori.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GporiParser : public MessageParser<bynav_gps_msgs::GporiPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GporiPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPORI_H
