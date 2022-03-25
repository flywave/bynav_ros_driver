#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gphpd.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GphpdParser::MESSAGE_NAME = "GPHPD";

uint32_t bynav_gps_driver::GphpdParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GphpdParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GphpdPtr bynav_gps_driver::GphpdParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPHPD log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GphpdPtr msg = boost::make_shared<bynav_gps_msgs::Gphpd>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[1], msg->week);
  valid = valid && ParseDouble(sentence.body[2], msg->seconds);

  valid = valid && ParseDouble(sentence.body[3], msg->yaw);
  valid = valid && ParseDouble(sentence.body[4], msg->pitch);
  valid = valid && ParseDouble(sentence.body[5], msg->heading);

  valid = valid && ParseDouble(sentence.body[6], msg->lat);
  valid = valid && ParseDouble(sentence.body[7], msg->lon);
  valid = valid && ParseDouble(sentence.body[8], msg->azimuth);

  valid = valid && ParseFloat(sentence.body[9], msg->east_distance);
  valid = valid && ParseFloat(sentence.body[10], msg->north_distance);
  valid = valid && ParseFloat(sentence.body[11], msg->up_distance);

  valid = valid && ParseDouble(sentence.body[12], msg->east_velocity);
  valid = valid && ParseDouble(sentence.body[13], msg->north_velocity);
  valid = valid && ParseDouble(sentence.body[14], msg->up_velocity);

  valid = valid && ParseFloat(sentence.body[15], msg->east_velocity_diff);
  valid = valid && ParseFloat(sentence.body[16], msg->north_velocity_diff);
  valid = valid && ParseFloat(sentence.body[17], msg->up_velocity_diff);

  valid = valid && ParseFloat(sentence.body[18], msg->baseline_length);

  valid = valid && ParseUInt8(sentence.body[19], msg->num_satellites_ant1);
  valid = valid && ParseUInt8(sentence.body[20], msg->num_satellites_ant2);

  uint8_t solution_status;
  valid = valid && ParseUInt8(sentence.body[21], solution_status);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPHPD");
  }

  return msg;
}
