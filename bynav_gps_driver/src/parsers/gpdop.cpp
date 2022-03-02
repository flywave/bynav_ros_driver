#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpdop.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GpdopParser::MESSAGE_NAME = "GPDOP";

uint32_t bynav_gps_driver::GpdopParser::GetMessageId() const {
  return 0;
}

const std::string bynav_gps_driver::GpdopParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpdopPtr bynav_gps_driver::GpdopParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPDOP log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpdopPtr msg = boost::make_shared<bynav_gps_msgs::Gpdop>();
  msg->message_id = sentence.body[0];
  ParseFloat(sentence.body[1], msg->utc_seconds);

  ParseFloat(sentence.body[2], msg->pdop);
  ParseFloat(sentence.body[3], msg->hdop);
  ParseFloat(sentence.body[4], msg->vdop);
  ParseFloat(sentence.body[5], msg->tdop);
  ParseFloat(sentence.body[6], msg->gdop);

  return msg;
}
