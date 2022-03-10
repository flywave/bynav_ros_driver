#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/marktime.h>
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
  msg->message_id = sentence.body[0];

  bool valid = true;

  if (!valid) {
    throw ParseException("Error parsing heading as double in MARKTIME");
  }

  return msg;
}
