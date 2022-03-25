#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/marktime.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::MarkTimeParser::MESSAGE_NAME = "MARKTIME";

uint32_t bynav_gps_driver::MarkTimeParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string bynav_gps_driver::MarkTimeParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::MarkTimePtr bynav_gps_driver::MarkTimeParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected inspva message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  
  bynav_gps_msgs::MarkTimePtr ros_msg =
      boost::make_shared<bynav_gps_msgs::MarkTime>();

  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->offset = ParseDouble(&bin_msg.data_[12]);
  ros_msg->offset_std = ParseDouble(&bin_msg.data_[20]);
  ros_msg->utc_offset = ParseDouble(&bin_msg.data_[28]);

  uint32_t status = ParseUInt32(&bin_msg.data_[36]);

  switch (status) {
  case 0:
    ros_msg->status = "VALID";
    break;
  case 1:
    ros_msg->status = "CONVERGING";
    break;
  case 2:
    ros_msg->status = "ITERATING";
    break;
  case 3:
    ros_msg->status = "INVALID";
    break;
  default: {
    std::stringstream error;
    error << "Unexpected inertial solution status: " << status;
    throw ParseException(error.str());
  }
  }
  return ros_msg;
}

bynav_gps_msgs::MarkTimePtr bynav_gps_driver::MarkTimeParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected MARKTIME length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::MarkTimePtr msg =
      boost::make_shared<bynav_gps_msgs::MarkTime>();

  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = GetMessageName();

  bool valid = true;
  valid &= ParseUInt32(sentence.body[0], msg->week);
  valid &= ParseDouble(sentence.body[1], msg->seconds);
  valid &= ParseDouble(sentence.body[2], msg->offset);
  valid &= ParseDouble(sentence.body[3], msg->offset_std);
  valid &= ParseDouble(sentence.body[4], msg->utc_offset);

  msg->status = sentence.body[5];

  if (!valid) {
    throw ParseException("Error parsing heading as double in MARKTIME");
  }

  return msg;
}
