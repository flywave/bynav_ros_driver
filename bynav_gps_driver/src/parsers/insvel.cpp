#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/insvel.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::InsvelParser::MESSAGE_NAME = "INSVEL";

uint32_t bynav_gps_driver::InsvelParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::InsvelParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InsvelPtr bynav_gps_driver::InsvelParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected INSVEL length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::InsvelPtr msg = boost::make_shared<bynav_gps_msgs::Insvel>();
  msg->message_id = sentence.body[0];

  double heading;
  if (swri_string_util::ToDouble(sentence.body[1], heading)) {
    msg->heading = heading;
  } else {
    throw ParseException("Error parsing heading as double in INSVEL");
  }

  return msg;
}
