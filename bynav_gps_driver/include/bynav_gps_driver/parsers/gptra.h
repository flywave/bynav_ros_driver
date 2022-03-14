#ifndef BYNAV_GPTRA_H
#define BYNAV_GPTRA_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gptra.h>

namespace bynav_gps_driver {

class GptraParser : public MessageParser<bynav_gps_msgs::GptraPtr> {
public:
  GptraParser()
      : MessageParser<bynav_gps_msgs::GptraPtr>(), was_last_gps_valid_(false) {}

  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GptraPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPTRA_H
