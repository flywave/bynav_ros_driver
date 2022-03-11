#ifndef BYNAV_GPZDA_H
#define BYNAV_GPZDA_H

#include <bynav_gps_msgs/Gpzda.h>

#include "message_parser.h"

namespace bynav_gps_driver {

class GpzdaParser : public MessageParser<bynav_gps_msgs::GpzdaPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpzdaPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPZDA_H
