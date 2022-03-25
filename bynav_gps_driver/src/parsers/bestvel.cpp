#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/bestvel.h>
#include <bynav_gps_driver/parsers/header.h>

const std::string bynav_gps_driver::BynavVelocityParser::MESSAGE_NAME =
    "BESTGNSSVEL";

uint32_t bynav_gps_driver::BynavVelocityParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string
bynav_gps_driver::BynavVelocityParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::BynavVelocityPtr
bynav_gps_driver::BynavVelocityParser::ParseBinary(
    const BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected velocity message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::BynavVelocityPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::BynavVelocity>();
  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = MESSAGE_NAME;

  uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
  if (solution_status > MAX_SOLUTION_STATUS) {
    std::stringstream error;
    error << "Unknown solution status: " << solution_status;
    throw ParseException(error.str());
  }
  ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
  uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
  if (pos_type > MAX_POSITION_TYPE) {
    std::stringstream error;
    error << "Unknown position type: " << pos_type;
    throw ParseException(error.str());
  }
  ros_msg->velocity_type = POSITION_TYPES[pos_type];
  ros_msg->latency = ParseFloat(&bin_msg.data_[8]);
  ros_msg->age = ParseFloat(&bin_msg.data_[12]);
  ros_msg->horizontal_speed = ParseDouble(&bin_msg.data_[16]);
  ros_msg->track_gnd = ParseDouble(&bin_msg.data_[24]);
  ros_msg->vertical_speed = ParseDouble(&bin_msg.data_[32]);

  return ros_msg;
}

bynav_gps_msgs::BynavVelocityPtr
bynav_gps_driver::BynavVelocityParser::ParseAscii(
    const BynavSentence &sentence) {
  bynav_gps_msgs::BynavVelocityPtr msg =
      boost::make_shared<bynav_gps_msgs::BynavVelocity>();
      
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  if (sentence.body.size() != ASCII_LENGTH) {
    std::stringstream error;
    error << "Unexpected number of BESTGNSSVEL message fields: "
          << sentence.body.size();
    throw ParseException(error.str());
  }
  bool valid = true;
  msg->solution_status = sentence.body[0];
  msg->velocity_type = sentence.body[1];
  valid = valid && ParseFloat(sentence.body[2], msg->latency);
  valid = valid && ParseFloat(sentence.body[3], msg->age);
  valid = valid && ParseDouble(sentence.body[4], msg->horizontal_speed);
  valid = valid && ParseDouble(sentence.body[5], msg->track_gnd);
  valid = valid && ParseDouble(sentence.body[6], msg->vertical_speed);

  if (!valid) {
    throw ParseException("Invalid field in BESTGNSSVEL message");
  }

  return msg;
}
