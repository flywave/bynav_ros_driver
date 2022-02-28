#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/inspos.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::InsposParser::MESSAGE_NAME = "INSPOS";

uint32_t bynav_gps_driver::InsposParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::InsposParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InsposPtr bynav_gps_driver::InsposParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected INSPOS length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::InsposPtr msg = boost::make_shared<bynav_gps_msgs::Inspos>();
  msg->message_id = sentence.body[0];

  double heading;
  if (swri_string_util::ToDouble(sentence.body[1], heading)) {
    msg->heading = heading;
  } else {
    throw ParseException("Error parsing heading as double in INSPOS");
  }

  return msg;
}
