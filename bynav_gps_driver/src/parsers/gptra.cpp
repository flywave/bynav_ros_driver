#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gptra.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::GptraParser::MESSAGE_NAME = "GPTRA";

uint32_t bynav_gps_driver::GptraParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GptraParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GptraPtr bynav_gps_driver::GptraParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 9;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected GPTRA length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GptraPtr msg = boost::make_shared<bynav_gps_msgs::Gptra>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);

  valid = valid && ParseDouble(sentence.body[2], msg->yaw);
  valid = valid && ParseDouble(sentence.body[3], msg->pitch);
  valid = valid && ParseDouble(sentence.body[4], msg->roll);

  valid = valid && ParseUInt32(sentence.body[5], msg->solution_status);

  valid = valid &&
          ParseUInt8(sentence.body[6], msg->num_satellites_used_in_solution);
  valid = valid && ParseFloat(sentence.body[7], msg->solution_age);

  msg->base_station_id = sentence.body[8];

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPTRA");
  }

  return msg;
}
