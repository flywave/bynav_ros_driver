#include <bynav_gps_driver/parsers/header.h>

#include <ros/ros.h>

uint32_t bynav_gps_driver::HeaderParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::HeaderParser::GetMessageName() const {
  return "HEADER";
}

bynav_gps_msgs::BynavMessageHeader bynav_gps_driver::HeaderParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) {
  bynav_gps_msgs::BynavMessageHeader msg;

  msg.port = PORT_IDENTIFIERS[bin_msg.header_.port_address_];
  msg.sequence_num = bin_msg.header_.sequence_;
  msg.percent_idle_time = bin_msg.header_.idle_time_;
  switch (bin_msg.header_.time_status_) {
  case 20:
    msg.gps_time_status = "UNKNOWN";
    break;
  case 60:
    msg.gps_time_status = "APPROXIMATE";
    break;
  case 80:
    msg.gps_time_status = "COARSEADJUSTING";
    break;
  case 100:
    msg.gps_time_status = "COARSE";
    break;
  case 120:
    msg.gps_time_status = "COARSESTEERING";
    break;
  case 130:
    msg.gps_time_status = "FREEWHEELING";
    break;
  case 140:
    msg.gps_time_status = "FINEADJUSTING";
    break;
  case 160:
    msg.gps_time_status = "FINE";
    break;
  case 170:
    msg.gps_time_status = "FINEBACKUPSTEERING";
    break;
  case 180:
    msg.gps_time_status = "FINESTEERING";
    break;
  case 200:
    msg.gps_time_status = "SATTIME";
    break;
  default: {
    std::stringstream error;
    error << "Unknown GPS time status: " << bin_msg.header_.time_status_;
    throw ParseException(error.str());
  }
  }
  msg.gps_week_num = bin_msg.header_.week_;
  msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;
  GetBynavReceiverStatusMessage(bin_msg.header_.receiver_status_,
                                msg.receiver_status);
  msg.receiver_software_version = bin_msg.header_.receiver_sw_version_;

  return msg;
}

bynav_gps_msgs::BynavMessageHeader
bynav_gps_driver::HeaderParser::ParseBinary(const BinaryMicroMessage &bin_msg) {
  bynav_gps_msgs::BynavMessageHeader msg;

  msg.gps_time_status = "UNKNOWN";
  msg.gps_week_num = bin_msg.header_.week_;
  msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;

  return msg;
}

bynav_gps_msgs::BynavMessageHeader bynav_gps_driver::HeaderParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) {
  if (sentence.header.size() != BYNAV_MESSAGE_HEADER_LENGTH) {
    std::stringstream error;
    error << "Bynav message header size wrong: expected "
          << BYNAV_MESSAGE_HEADER_LENGTH << ", got %zu"
          << sentence.header.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  bynav_gps_msgs::BynavMessageHeader msg;
  msg.message_name = sentence.header[0];
  msg.port = sentence.header[1];
  valid = valid && ParseUInt32(sentence.header[2], msg.sequence_num);
  valid = valid && ParseFloat(sentence.header[3], msg.percent_idle_time);
  msg.gps_time_status = sentence.header[4];
  valid = valid && ParseUInt32(sentence.header[5], msg.gps_week_num);
  valid = valid && ParseDouble(sentence.header[6], msg.gps_seconds);

  uint32_t receiver_status_code = 0;
  valid = valid && ParseUInt32(sentence.header[7], receiver_status_code, 16);
  GetBynavReceiverStatusMessage(receiver_status_code, msg.receiver_status);

  valid =
      valid && ParseUInt32(sentence.header[9], msg.receiver_software_version);

  if (!valid) {
    throw ParseException("Header was invalid.");
  }
  return msg;
}
