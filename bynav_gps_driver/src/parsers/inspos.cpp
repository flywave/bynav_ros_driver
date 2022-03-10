#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/inspos.h>
#include <bynav_gps_driver/parsers/header.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::InsposParser::MESSAGE_NAME = "INSPOS";

uint32_t bynav_gps_driver::InsposParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::InsposParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InsposPtr bynav_gps_driver::InsposParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 7;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected INSPOS length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::InsposPtr msg = boost::make_shared<bynav_gps_msgs::Inspos>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = GetMessageName();

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[1], msg->week);
  valid = valid && ParseDouble(sentence.body[2], msg->seconds);
  valid = valid && ParseDouble(sentence.body[3], msg->latitude);
  valid = valid && ParseDouble(sentence.body[4], msg->longitude);
  valid = valid && ParseDouble(sentence.body[5], msg->height);

  msg->status = sentence.body[6];

  if (!valid) {
    throw ParseException("Error parsing heading as double in INSPOS");
  }

  return msg;
}
