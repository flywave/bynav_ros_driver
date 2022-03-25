#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpvtg.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::GpvtgParser::MESSAGE_NAME = "GPVTG";

uint32_t bynav_gps_driver::GpvtgParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpvtgParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpvtgPtr bynav_gps_driver::GpvtgParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected GPVTG length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpvtgPtr msg = boost::make_shared<bynav_gps_msgs::Gpvtg>();

  msg->message_id = sentence.body[0];

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[1], msg->track);
  msg->t = sentence.body[2];
  valid = valid && ParseDouble(sentence.body[3], msg->mtrack);
  msg->mt = sentence.body[4];
  valid = valid && ParseDouble(sentence.body[5], msg->horizontal_speedn);
  msg->spn_nt_unit = sentence.body[6];
  valid = valid && ParseDouble(sentence.body[7], msg->horizontal_speedk);
  msg->spn_kt_unit = sentence.body[8];
  msg->status = sentence.body[9];

  if (!valid) {
    throw ParseException("Error parsing heading as double in GPVTG");
  }

  return msg;
}
