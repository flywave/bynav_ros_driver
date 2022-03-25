#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/inspos.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::InsposParser::MESSAGE_NAME = "INSPOS";

uint32_t bynav_gps_driver::InsposParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::InsposParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InsposPtr bynav_gps_driver::InsposParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected inspva message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::InsposPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::Inspos>();
  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);

  ros_msg->latitude = ParseDouble(&bin_msg.data_[12]);
  ros_msg->longitude = ParseDouble(&bin_msg.data_[20]);
  ros_msg->height = ParseDouble(&bin_msg.data_[28]);

  uint32_t status = ParseUInt32(&bin_msg.data_[36]);

  switch (status) {
  case 0:
    ros_msg->status = "INS_INACTIVE";
    break;
  case 1:
    ros_msg->status = "INS_ALIGNING";
    break;
  case 2:
    ros_msg->status = "INS_HIGH_VARIANCE";
    break;
  case 3:
    ros_msg->status = "INS_SOLUTION_GOOD";
    break;
  case 6:
    ros_msg->status = "INS_SOLUTION_FREE";
    break;
  case 7:
    ros_msg->status = "INS_ALIGNMENT_COMPLETE";
    break;
  case 8:
    ros_msg->status = "DETERMINING_ORIENTATION";
    break;
  case 9:
    ros_msg->status = "WAITING_INITIALPOS";
    break;
  case 10:
    ros_msg->status = "WAITING_AZIMUTH";
    break;
  case 11:
    ros_msg->status = "INITIALIZING_BASES";
    break;
  case 12:
    ros_msg->status = "MOTION_DETECT";
    break;
  default: {
    std::stringstream error;
    error << "Unexpected inertial solution status: " << status;
    throw ParseException(error.str());
  }
  }
  return ros_msg;
}

bynav_gps_msgs::InsposPtr bynav_gps_driver::InsposParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  const size_t EXPECTED_LEN = 7;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected INSPOS length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::InsposPtr msg = boost::make_shared<bynav_gps_msgs::Inspos>();

  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  msg->bynav_msg_header.message_name = GetMessageName();

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[0], msg->week);
  valid = valid && ParseDouble(sentence.body[1], msg->seconds);
  valid = valid && ParseDouble(sentence.body[2], msg->latitude);
  valid = valid && ParseDouble(sentence.body[3], msg->longitude);
  valid = valid && ParseDouble(sentence.body[4], msg->height);

  msg->status = sentence.body[5];

  if (!valid) {
    throw ParseException("Error parsing heading as double in INSPOS");
  }

  return msg;
}
