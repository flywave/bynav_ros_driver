#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpzda.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::GpzdaParser::MESSAGE_NAME = "GPZDA";

uint32_t bynav_gps_driver::GpzdaParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpzdaParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpzdaPtr bynav_gps_driver::GpzdaParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 7;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected GPZDA length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpzdaPtr msg = boost::make_shared<bynav_gps_msgs::Gpzda>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);
  valid = valid && ParseUInt32(sentence.body[2], msg->utc_year);
  valid = valid && ParseUInt8(sentence.body[3], msg->utc_month);
  valid = valid && ParseUInt8(sentence.body[4], msg->utc_day);
  valid = valid && ParseUInt8(sentence.body[5], msg->local_zone);
  valid = valid && ParseUInt32(sentence.body[6], msg->local_zone_minutes);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPZDA");
  }

  return msg;
}
