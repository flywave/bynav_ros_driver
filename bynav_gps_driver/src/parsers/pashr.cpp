#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/pashr.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::PashrParser::MESSAGE_NAME = "PASHR";

uint32_t bynav_gps_driver::PashrParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::PashrParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::PashrPtr bynav_gps_driver::PashrParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 11;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected PASHR length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::PashrPtr msg = boost::make_shared<bynav_gps_msgs::Pashr>();

  bool valid = true;

  msg->message_id = sentence.body[0];

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);
  valid = valid && ParseDouble(sentence.body[2], msg->heading);

  msg->t = sentence.body[3];

  valid = valid && ParseDouble(sentence.body[4], msg->yaw);
  valid = valid && ParseDouble(sentence.body[5], msg->pitch);

  valid = valid && ParseFloat(sentence.body[6], msg->undulation);

  valid = valid && ParseFloat(sentence.body[7], msg->yaw_std);
  valid = valid && ParseFloat(sentence.body[8], msg->pitch_std);
  valid = valid && ParseFloat(sentence.body[9], msg->heading_std);

  msg->solution_status = sentence.body[10];

  if (!valid) {
    throw ParseException("Error parsing heading as double in PASHR");
  }

  return msg;
}
