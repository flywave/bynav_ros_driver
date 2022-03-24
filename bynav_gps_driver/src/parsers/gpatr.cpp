#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpatr.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GpatrParser::MESSAGE_NAME = "GPATR";

uint32_t bynav_gps_driver::GpatrParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpatrParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpatrPtr bynav_gps_driver::GpatrParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t LENGTH = 11;
  if (sentence.body.size() < LENGTH) {
    std::stringstream error;
    error << "Expected GPATR length " << LENGTH << ", actual length "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpatrPtr msg = boost::make_shared<bynav_gps_msgs::Gpatr>();
  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);
  msg->position_status = sentence.body[2];

  valid = valid && ParseFloat(sentence.body[3], msg->baseline_length);

  valid = valid && ParseFloat(sentence.body[4], msg->north_distance);
  valid = valid && ParseFloat(sentence.body[5], msg->east_distance);
  valid = valid && ParseFloat(sentence.body[6], msg->up_distance);

  msg->orientation_type = sentence.body[7];

  valid = valid && ParseFloat(sentence.body[8], msg->yaw);
  valid = valid && ParseFloat(sentence.body[9], msg->pitch);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPATR");
  }

  return msg;
}
