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
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPFPD log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpdopPtr msg = boost::make_shared<bynav_gps_msgs::Gpdop>();
  msg->message_id = sentence.body[0];
  ParseUInt32(sentence.body[1], msg->week);
  ParseFloat(sentence.body[2], msg->seconds);

  ParseDouble(sentence.body[3], msg->yaw);
  ParseDouble(sentence.body[4], msg->pitch);
  ParseDouble(sentence.body[5], msg->roll);

  ParseDouble(sentence.body[6], msg->lat);
  ParseDouble(sentence.body[7], msg->lon);
  ParseDouble(sentence.body[8], msg->azimuth);

  ParseDouble(sentence.body[9], msg->east_velocity);
  ParseDouble(sentence.body[10], msg->north_velocity);
  ParseDouble(sentence.body[11], msg->up_velocity);

  ParseDouble(sentence.body[12], msg->baseline_length);

  ParseUInt8(sentence.body[13], msg->num_satellites_ant1);
  ParseUInt8(sentence.body[14], msg->num_satellites_ant2);

  uint8_t solution_status;
  ParseUInt8(sentence.body[15], solution_status);

  return msg;
}
