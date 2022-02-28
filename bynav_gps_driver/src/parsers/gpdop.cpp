#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpdop.h>
#include <bynav_gps_driver/parsers/header.h>
#include <sstream>

const std::string bynav_gps_driver::GpdopParser::MESSAGE_NAME = "GPDOP";

uint32_t bynav_gps_driver::GpdopParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string bynav_gps_driver::GpdopParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpdopPtr bynav_gps_driver::GpdopParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) {
  uint32_t num_systems = ParseUInt32(&bin_msg.data_[16]);
  if (bin_msg.data_.size() !=
      (BINARY_SYSTEM_LENGTH * num_systems) + BINARY_BODY_LENGTH) {
    std::stringstream error;
    error << "Unexpected GPDOP message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }

  auto ros_msg = boost::make_shared<bynav_gps_msgs::Gpdop>();

  HeaderParser header_parser;
  ros_msg->bynav_msg_header = header_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = MESSAGE_NAME;

  ros_msg->gdop = ParseFloat(&bin_msg.data_[0]);

  ros_msg->pdop = ParseFloat(&bin_msg.data_[4]);

  ros_msg->hdop = ParseFloat(&bin_msg.data_[8]);

  ros_msg->vdop = ParseFloat(&bin_msg.data_[12]);

  return ros_msg;
}

bynav_gps_msgs::GpdopPtr bynav_gps_driver::GpdopParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.body.size() < ASCII_BODY_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of body fields in GPDOP log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  uint32_t num_systems = 0;
  ParseUInt32(sentence.body[4], num_systems);

  if (sentence.body.size() !=
      ASCII_BODY_FIELDS + num_systems * ASCII_SYSTEM_FIELDS) {
    std::stringstream error;
    error << "Size of GPDOP log (" << sentence.body.size()
          << ") did not match expected size ("
          << ASCII_BODY_FIELDS + num_systems * ASCII_SYSTEM_FIELDS << ").";
    throw ParseException(error.str());
  }

  bool valid = true;
  auto msg = boost::make_shared<bynav_gps_msgs::Gpdop>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  valid &= ParseFloat(sentence.body[0], msg->gdop);
  valid &= ParseFloat(sentence.body[1], msg->pdop);
  valid &= ParseFloat(sentence.body[2], msg->hdop);
  valid &= ParseFloat(sentence.body[3], msg->vdop);

  if (!valid) {
    std::stringstream error;
    error << "Error parsing GPDOP log.";
    throw ParseException(error.str());
  }
  return msg;
}
