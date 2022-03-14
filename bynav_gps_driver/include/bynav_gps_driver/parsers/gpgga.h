#ifndef BYNAV_GPGGA_H
#define BYNAV_GPGGA_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gpgga.h>

namespace bynav_gps_driver {

class GpggaParser : public MessageParser<bynav_gps_msgs::GpggaPtr> {
public:
  GpggaParser()
      : MessageParser<bynav_gps_msgs::GpggaPtr>(), was_last_gps_valid_(false) {}
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GpggaPtr ParseAscii(const NmeaSentence &sentence) override;

  bool WasLastGpsValid() const;

  static const std::string MESSAGE_NAME;

private:
  bool was_last_gps_valid_;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPGGA_H
