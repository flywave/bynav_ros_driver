#include <bynav_gps_driver/parsers/rawimu.h>

#include <bynav_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace bynav_gps_driver {

const std::string RawIMUParser::MESSAGE_NAME = "RAWIMU";

uint32_t RawIMUParser::GetMessageId() const { return 0; }

const std::string RawIMUParser::GetMessageName() const { return MESSAGE_NAME; }

bynav_gps_msgs::RawIMUPtr bynav_gps_driver::RawIMUParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected corrimudata message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  
  bynav_gps_msgs::RawIMUPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::RawIMU>();

  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = "RAWIMU";

  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->imu_status = ParseUInt32(&bin_msg.data_[12]);

  ros_msg->z_accel = ParseInt32(&bin_msg.data_[16]);
  ros_msg->y_accel = ParseInt32(&bin_msg.data_[20]);
  ros_msg->x_accel = ParseInt32(&bin_msg.data_[24]);

  ros_msg->z_gyro = ParseInt32(&bin_msg.data_[28]);
  ros_msg->y_gyro = ParseInt32(&bin_msg.data_[32]);
  ros_msg->x_gyro = ParseInt32(&bin_msg.data_[36]);

  return ros_msg;
}

bynav_gps_msgs::RawIMUPtr bynav_gps_driver::RawIMUParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.body.size() != ASCII_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of fields in RAWIMUA log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::RawIMUPtr msg = boost::make_shared<bynav_gps_msgs::RawIMU>();

  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);

  valid &= ParseUInt32(sentence.body[2], msg->imu_status);

  valid &= ParseInt32(sentence.body[3], msg->z_accel);
  valid &= ParseInt32(sentence.body[4], msg->y_accel);
  valid &= ParseInt32(sentence.body[5], msg->x_accel);

  valid &= ParseInt32(sentence.body[6], msg->z_gyro);
  valid &= ParseInt32(sentence.body[7], msg->y_gyro);
  valid &= ParseInt32(sentence.body[8], msg->x_gyro);

  if (!valid) {
    throw ParseException("Error parsing RAWIMUA log.");
  }

  return msg;
}

} // namespace bynav_gps_driver
