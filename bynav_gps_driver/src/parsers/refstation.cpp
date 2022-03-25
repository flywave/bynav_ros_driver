#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/refstation.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::RefStationParser::MESSAGE_NAME =
    "REFSTATIONA";

uint32_t bynav_gps_driver::RefStationParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::RefStationParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::RefStationPtr bynav_gps_driver::RefStationParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected REFSTATIONA length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::RefStationPtr msg =
      boost::make_shared<bynav_gps_msgs::RefStation>();
      
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = "REFSTATIONA";

  double b_x;
  if (swri_string_util::ToDouble(sentence.body[1], b_x)) {
    msg->ecef_x = b_x;
  } else {
    throw ParseException("Error parsing heading as double in REFSTATIONA");
  }

  double l_y;
  if (swri_string_util::ToDouble(sentence.body[2], l_y)) {
    msg->ecef_y = l_y;
  } else {
    throw ParseException("Error parsing heading as double in REFSTATIONA");
  }

  double h_z;
  if (swri_string_util::ToDouble(sentence.body[3], h_z)) {
    msg->ecef_z = h_z;
  } else {
    throw ParseException("Error parsing heading as double in REFSTATIONA");
  }

  return msg;
}
