#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/ksxt.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::BynavKsxtParser::MESSAGE_NAME = "KSXT";

uint32_t bynav_gps_driver::BynavKsxtParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::BynavKsxtParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::BynavKsxtPtr bynav_gps_driver::BynavKsxtParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected KSXT length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::BynavKsxtPtr msg =
      boost::make_shared<bynav_gps_msgs::BynavKsxt>();
  msg->message_id = sentence.body[0];

  double heading;
  if (swri_string_util::ToDouble(sentence.body[1], heading)) {
    msg->heading = heading;
  } else {
    throw ParseException("Error parsing heading as double in KSXT");
  }

  return msg;
}
