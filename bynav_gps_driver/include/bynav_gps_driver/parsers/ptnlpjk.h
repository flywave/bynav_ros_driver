#ifndef BYNAV_BESTXYZ_H
#define BYNAV_BESTXYZ_H

#include <bynav_gps_msgs/PtnlPJK.h>

#include <bynav_gps_driver/parsers/message_parser.h>
#include <bynav_gps_driver/parsers/parsing_utils.h>

namespace bynav_gps_driver {

class PtnlPJKParser : public MessageParser<bynav_gps_msgs::PtnlPJKPtr> {
public:
  uint32_t GetMessageId() const override;

  const std::string GetMessageName() const override;

  bynav_gps_msgs::PtnlPJKPtr ParseAscii(const NmeaSentence &sentence) override;

  static const std::string MESSAGE_NAME;
};
} // namespace bynav_gps_driver

#endif // BYNAV_BESTXYZ_H
