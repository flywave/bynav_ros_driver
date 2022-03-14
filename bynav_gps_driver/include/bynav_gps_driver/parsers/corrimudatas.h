#ifndef BYNAV_CORRIMUDATAS_H
#define BYNAV_CORRIMUDATAS_H

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_msgs/BynavCorrectedImuData.h>

namespace bynav_gps_driver {

class CorrImuDataSParser
    : public MessageParser<bynav_gps_msgs::BynavCorrectedImuDataPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::BynavCorrectedImuDataPtr
  ParseBinary(const BinaryMicroMessage &bin_msg) override;

  bynav_gps_msgs::BynavCorrectedImuDataPtr
  ParseAscii(const BynavSentence &sentence) override;

  static constexpr uint16_t MESSAGE_ID = 813;
  static constexpr size_t BINARY_LENGTH = 60;
  static constexpr size_t ASCII_FIELDS = 8;
  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_CORRIMUDATAS_H
