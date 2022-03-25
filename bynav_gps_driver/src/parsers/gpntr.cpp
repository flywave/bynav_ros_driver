#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpntr.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GpntrParser::MESSAGE_NAME = "GPNTR";

uint32_t bynav_gps_driver::GpntrParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpntrParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpntrPtr bynav_gps_driver::GpntrParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPNTR log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpntrPtr msg = boost::make_shared<bynav_gps_msgs::Gpntr>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);
  valid = valid && ParseUInt32(sentence.body[2], msg->solution_status);

  valid = valid && ParseFloat(sentence.body[3], msg->distance);

  valid = valid && ParseFloat(sentence.body[4], msg->x_distance);
  valid = valid && ParseFloat(sentence.body[5], msg->y_distance);
  valid = valid && ParseFloat(sentence.body[6], msg->z_distance);

  msg->base_station_id = sentence.body[7];

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPNTR");
  }

  return msg;
}
