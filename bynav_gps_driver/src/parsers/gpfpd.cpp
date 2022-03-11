#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpdop.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GpfpdParser::MESSAGE_NAME = "GPFPD";

uint32_t bynav_gps_driver::GpfpdParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpfpdParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpfpdPtr bynav_gps_driver::GpfpdParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPFPD log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpfpdPtr msg = boost::make_shared<bynav_gps_msgs::Gpfpd>();
  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[0], msg->week);
  valid = valid && ParseFloat(sentence.body[1], msg->seconds);

  valid = valid && ParseDouble(sentence.body[2], msg->yaw);
  valid = valid && ParseDouble(sentence.body[3], msg->pitch);
  valid = valid && ParseDouble(sentence.body[4], msg->roll);

  valid = valid && ParseDouble(sentence.body[5], msg->lat);
  valid = valid && ParseDouble(sentence.body[6], msg->lon);
  valid = valid && ParseDouble(sentence.body[7], msg->azimuth);

  valid = valid && ParseDouble(sentence.body[8], msg->east_velocity);
  valid = valid && ParseDouble(sentence.body[9], msg->north_velocity);
  valid = valid && ParseDouble(sentence.body[10], msg->up_velocity);

  valid = valid && ParseDouble(sentence.body[11], msg->baseline_length);

  valid = valid && ParseUInt8(sentence.body[12], msg->num_satellites_ant1);
  valid = valid && ParseUInt8(sentence.body[13], msg->num_satellites_ant2);

  uint8_t solution_status;
  valid = valid && ParseUInt8(sentence.body[14], solution_status);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPFPD");
  }

  return msg;
}
