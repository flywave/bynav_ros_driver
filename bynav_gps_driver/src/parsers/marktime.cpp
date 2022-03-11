#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/marktime.h>
#include <bynav_gps_driver/parsers/header.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::MarkTimeParser::MESSAGE_NAME = "MARKTIME";

uint32_t bynav_gps_driver::MarkTimeParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::MarkTimeParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::MarkTimePtr bynav_gps_driver::MarkTimeParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected MARKTIME length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::MarkTimePtr msg =
      boost::make_shared<bynav_gps_msgs::MarkTime>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = GetMessageName();

  bool valid = true;
  valid &= ParseUInt32(sentence.body[0], msg->week);
  valid &= ParseDouble(sentence.body[2], msg->seconds);
  valid &= ParseDouble(sentence.body[3], msg->offset);
  valid &= ParseDouble(sentence.body[4], msg->offset_std);
  valid &= ParseDouble(sentence.body[5], msg->utc_offset);

  msg->status = sentence.body[6];

  if (!valid) {
    throw ParseException("Error parsing heading as double in MARKTIME");
  }

  return msg;
}
