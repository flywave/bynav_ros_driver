#ifndef BYNAV_GPRMC_H
#define BYNAV_GPRMC_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/Gprmc.h>

namespace bynav_gps_driver {

class GprmcParser : public MessageParser<bynav_gps_msgs::GprmcPtr> {
public:
  GprmcParser()
      : MessageParser<bynav_gps_msgs::GprmcPtr>(), was_last_gps_valid_(false) {}

  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::GprmcPtr ParseAscii(const NmeaSentence &sentence) override;

  bool WasLastGpsValid() const;

  static const std::string MESSAGE_NAME;
  static constexpr double KNOTS_TO_MPS = 0.5144444;

private:
  bool was_last_gps_valid_;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPRMC_H
