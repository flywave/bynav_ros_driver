#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpgst.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::GpgstParser::MESSAGE_NAME = "GPGST";

uint32_t bynav_gps_driver::GpgstParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpgstParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpgstPtr bynav_gps_driver::GpgstParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 9;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected GPGST length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpgstPtr msg = boost::make_shared<bynav_gps_msgs::Gpgst>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->utc_seconds);

  valid = valid && ParseDouble(sentence.body[2], msg->rmssd);
  valid = valid && ParseDouble(sentence.body[3], msg->sdmaj);
  valid = valid && ParseDouble(sentence.body[4], msg->sdmin);
  valid = valid && ParseDouble(sentence.body[5], msg->orient);
  valid = valid && ParseDouble(sentence.body[6], msg->lat_std);
  valid = valid && ParseDouble(sentence.body[7], msg->lon_std);
  valid = valid && ParseDouble(sentence.body[8], msg->alt_std);

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPGST");
  }

  return msg;
}
