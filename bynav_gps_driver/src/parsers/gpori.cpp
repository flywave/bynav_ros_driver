#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpori.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::GporiParser::MESSAGE_NAME = "GPORI";

uint32_t bynav_gps_driver::GporiParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GporiParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GporiPtr bynav_gps_driver::GporiParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 9;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected GPORI length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GporiPtr msg = boost::make_shared<bynav_gps_msgs::Gpori>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);
  
  msg->position_status = sentence.body[2];

  valid = valid && ParseFloat(sentence.body[3], msg->baseline_length);

  valid = valid && ParseFloat(sentence.body[4], msg->heading);
  valid = valid && ParseFloat(sentence.body[5], msg->tilt);

  valid = valid && ParseFloat(sentence.body[6], msg->baseline_x);
  valid = valid && ParseFloat(sentence.body[7], msg->baseline_y);
  valid = valid && ParseFloat(sentence.body[8], msg->baseline_z);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPORI");
  }

  return msg;
}
