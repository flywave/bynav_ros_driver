#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/insatt.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::InsattParser::MESSAGE_NAME = "INSATT";

uint32_t bynav_gps_driver::InsattParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::InsattParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InsattPtr bynav_gps_driver::InsattParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 8;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected INSATT length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::InsattPtr msg = boost::make_shared<bynav_gps_msgs::Insatt>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = GetMessageName();

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[0], msg->week);
  valid = valid && ParseDouble(sentence.body[1], msg->seconds);

  valid = valid && ParseDouble(sentence.body[2], msg->roll);
  valid = valid && ParseDouble(sentence.body[3], msg->pitch);
  valid = valid && ParseDouble(sentence.body[4], msg->azimuth);

  msg->status = sentence.body[5];

  if (!valid) {
    throw ParseException("Error parsing INSATT log.");
  }

  return msg;
}
