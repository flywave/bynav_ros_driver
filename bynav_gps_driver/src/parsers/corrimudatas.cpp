#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/corrimudatas.h>
#include <bynav_gps_driver/parsers/header.h>

const std::string bynav_gps_driver::CorrImuDataSParser::MESSAGE_NAME =
    "CORRIMUDATAS";

uint32_t bynav_gps_driver::CorrImuDataSParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string bynav_gps_driver::CorrImuDataSParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::BynavCorrectedImuDataPtr
bynav_gps_driver::CorrImuDataSParser::ParseBinary(
    const bynav_gps_driver::BinaryMicroMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected corrimudata message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::BynavCorrectedImuDataPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::BynavCorrectedImuData>();
  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = "CORRIMUDATA";

  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->pitch_rate = ParseDouble(&bin_msg.data_[12]);
  ros_msg->roll_rate = ParseDouble(&bin_msg.data_[20]);
  ros_msg->yaw_rate = ParseDouble(&bin_msg.data_[28]);
  ros_msg->lateral_acceleration = ParseDouble(&bin_msg.data_[36]);
  ros_msg->longitudinal_acceleration = ParseDouble(&bin_msg.data_[44]);
  ros_msg->vertical_acceleration = ParseDouble(&bin_msg.data_[52]);

  return ros_msg;
}

bynav_gps_msgs::BynavCorrectedImuDataPtr
bynav_gps_driver::CorrImuDataSParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.body.size() != ASCII_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of fields in CORRIMUDATA log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::BynavCorrectedImuDataPtr msg =
      boost::make_shared<bynav_gps_msgs::BynavCorrectedImuData>();
  
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseDouble(sentence.body[2], msg->pitch_rate);
  valid &= ParseDouble(sentence.body[3], msg->roll_rate);
  valid &= ParseDouble(sentence.body[4], msg->yaw_rate);
  valid &= ParseDouble(sentence.body[5], msg->lateral_acceleration);
  valid &= ParseDouble(sentence.body[6], msg->longitudinal_acceleration);
  valid &= ParseDouble(sentence.body[7], msg->vertical_acceleration);

  if (!valid) {
    throw ParseException("Error parsing CORRIMUDATA log.");
  }

  return msg;
}
