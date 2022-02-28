#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/inspva.h>

const std::string bynav_gps_driver::InspvaParser::MESSAGE_NAME = "INSPVA";

uint32_t bynav_gps_driver::InspvaParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string bynav_gps_driver::InspvaParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::InspvaPtr bynav_gps_driver::InspvaParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) noexcept(false) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected inspva message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::InspvaPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::Inspva>();
  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->latitude = ParseDouble(&bin_msg.data_[12]);
  ros_msg->longitude = ParseDouble(&bin_msg.data_[20]);
  ros_msg->height = ParseDouble(&bin_msg.data_[28]);
  ros_msg->north_velocity = ParseDouble(&bin_msg.data_[36]);
  ros_msg->east_velocity = ParseDouble(&bin_msg.data_[44]);
  ros_msg->up_velocity = ParseDouble(&bin_msg.data_[52]);
  ros_msg->roll = ParseDouble(&bin_msg.data_[60]);
  ros_msg->pitch = ParseDouble(&bin_msg.data_[68]);
  ros_msg->azimuth = ParseDouble(&bin_msg.data_[76]);
  uint32_t status = ParseUInt32(&bin_msg.data_[84]);

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

bynav_gps_msgs::InspvaPtr bynav_gps_driver::InspvaParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  if (sentence.body.size() != ASCII_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of fields in INSPVA log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::InspvaPtr msg = boost::make_shared<bynav_gps_msgs::Inspva>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->week);
  valid &= ParseDouble(sentence.body[1], msg->seconds);
  valid &= ParseDouble(sentence.body[2], msg->latitude);
  valid &= ParseDouble(sentence.body[3], msg->longitude);
  valid &= ParseDouble(sentence.body[4], msg->height);
  valid &= ParseDouble(sentence.body[5], msg->north_velocity);
  valid &= ParseDouble(sentence.body[6], msg->east_velocity);
  valid &= ParseDouble(sentence.body[7], msg->up_velocity);
  valid &= ParseDouble(sentence.body[8], msg->roll);
  valid &= ParseDouble(sentence.body[9], msg->pitch);
  valid &= ParseDouble(sentence.body[10], msg->azimuth);
  msg->status = sentence.body[11];

  if (!valid) {
    throw ParseException("Error parsing INSPVA log.");
  }

  return msg;
}
