#ifndef BYNAV_GPS_DRIVER_CORRIMUDATA_H
#define BYNAV_GPS_DRIVER_CORRIMUDATA_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/BynavCorrectedImuData.h>

namespace bynav_gps_driver {

class CorrImuDataParser
    : public MessageParser<bynav_gps_msgs::BynavCorrectedImuDataPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavCorrectedImuDataPtr
  ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

  bynav_gps_msgs::BynavCorrectedImuDataPtr
  ParseAscii(const BynavSentence &sentence) noexcept(false) override;

  static constexpr uint16_t MESSAGE_ID = 812;
  static constexpr size_t BINARY_LENGTH = 60;
  static constexpr size_t ASCII_FIELDS = 8;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_GPS_DRIVER_CORRIMUDATA_H
