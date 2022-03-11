#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/mark2time.h>
#include <swri_string_util/string_util.h>

const std::string bynav_gps_driver::Mark2TimeParser::MESSAGE_NAME = "MARK2TIME";

uint32_t bynav_gps_driver::Mark2TimeParser::GetMessageId() const { return MESSAGE_ID; }

const std::string bynav_gps_driver::Mark2TimeParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::MarkTimePtr bynav_gps_driver::Mark2TimeParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) noexcept(false) {
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

  return ros_msg;
}

bynav_gps_msgs::MarkTimePtr bynav_gps_driver::Mark2TimeParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected MARK2TIME length = " << EXPECTED_LEN << ", "
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
  valid &= ParseDouble(sentence.body[2], msg->seconds);
  valid &= ParseDouble(sentence.body[3], msg->offset);
  valid &= ParseDouble(sentence.body[4], msg->offset_std);
  valid &= ParseDouble(sentence.body[5], msg->utc_offset);

  msg->status = sentence.body[6];

  if (!valid) {
    throw ParseException("Error parsing heading as double in MARK2TIME");
  }

  return msg;
}
