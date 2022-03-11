#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/enuavr.h>
#include <bynav_gps_driver/parsers/header.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::EnuavrParser::MESSAGE_NAME = "ENUAVR";

uint32_t bynav_gps_driver::EnuavrParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::EnuavrParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::BynavEnuAvrPtr bynav_gps_driver::EnuavrParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected ENUAVR length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::BynavEnuAvrPtr msg =
      boost::make_shared<bynav_gps_msgs::BynavEnuAvr>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = "ENUAVR";

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->ant1_north);
  valid = valid && ParseDouble(sentence.body[2], msg->ant1_up);
  valid = valid && ParseDouble(sentence.body[3], msg->ant2_east);
  valid = valid && ParseDouble(sentence.body[4], msg->ant2_north);
  valid = valid && ParseDouble(sentence.body[5], msg->ant2_up);

  valid = valid && ParseDouble(sentence.body[6], msg->roll);
  valid = valid && ParseDouble(sentence.body[7], msg->pitch);

  valid = valid && ParseUInt32(sentence.body[8], msg->count);

  if (!valid) {
    throw ParseException("Error parsing heading as double in ENUAVR");
  }

  return msg;
}
