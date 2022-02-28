#include <bynav_gps_driver/parsers/header.h>
#include <bynav_gps_driver/parsers/range.h>

#include <boost/make_shared.hpp>

const std::string bynav_gps_driver::RangeParser::MESSAGE_NAME = "RANGE";

uint32_t bynav_gps_driver::RangeParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string bynav_gps_driver::RangeParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::RangePtr bynav_gps_driver::RangeParser::ParseBinary(
    const bynav_gps_driver::BinaryMessage &bin_msg) noexcept(false) {
  uint32_t num_obs = ParseUInt32(&bin_msg.data_[0]);
  if (bin_msg.data_.size() != (BINARY_OBSERVATION_SIZE * num_obs) + 4) {
    std::stringstream error;
    error << "Unexpected range message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::RangePtr ros_msg =
      boost::make_shared<bynav_gps_msgs::Range>();
  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->bynav_msg_header.message_name = "RANGE";

  ros_msg->numb_of_observ = num_obs;
  ros_msg->info.reserve(num_obs);
  for (int i = 0; i < num_obs; i++) {
    size_t obs_offset = 4 + i * BINARY_OBSERVATION_SIZE;

    bynav_gps_msgs::RangeInformation info;

    info.prn_number = ParseUInt16(&bin_msg.data_[obs_offset]);
    info.glofreq = ParseUInt16(&bin_msg.data_[obs_offset + 2]);
    info.psr = ParseDouble(&bin_msg.data_[obs_offset + 4]);
    info.psr_std = ParseFloat(&bin_msg.data_[obs_offset + 12]);
    info.adr = ParseDouble(&bin_msg.data_[obs_offset + 16]);
    info.adr_std = ParseFloat(&bin_msg.data_[obs_offset + 24]);
    info.dopp = ParseFloat(&bin_msg.data_[obs_offset + 28]);
    info.noise_density_ratio = ParseFloat(&bin_msg.data_[obs_offset + 32]);
    info.locktime = ParseFloat(&bin_msg.data_[obs_offset + 36]);
    info.tracking_status = ParseUInt32(&bin_msg.data_[obs_offset + 40]);

    ros_msg->info.push_back(info);
  }
  return ros_msg;
}

bynav_gps_msgs::RangePtr bynav_gps_driver::RangeParser::ParseAscii(
    const bynav_gps_driver::BynavSentence &sentence) noexcept(false) {
  bynav_gps_msgs::RangePtr msg = boost::make_shared<bynav_gps_msgs::Range>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);
  if (!ParseInt32(sentence.body[0], msg->numb_of_observ, 10)) {
    std::stringstream error;
    error << "Unable to parse number of observations in RANGE log.";
    throw ParseException(error.str());
  }
  uint32_t numb_of_observ = static_cast<uint32_t>(msg->numb_of_observ);
  if (sentence.body.size() != 1 + numb_of_observ * ASCII_FIELDS) {
    std::stringstream error;
    error << "Did not find expected number of observations in RANGE log.";
    throw ParseException(error.str());
  }
  bool valid = true;
  valid &= ParseInt32(sentence.body[0], msg->numb_of_observ, 10);
  msg->info.resize(numb_of_observ);
  for (int i = 0, index = 0; index < numb_of_observ; i += 10, index++) {
    valid &= ParseUInt16(sentence.body[i + 1], msg->info[index].prn_number, 10);
    valid &= ParseUInt16(sentence.body[i + 2], msg->info[index].glofreq, 10);
    valid &= ParseDouble(sentence.body[i + 3], msg->info[index].psr);
    valid &= ParseFloat(sentence.body[i + 4], msg->info[index].psr_std);
    valid &= ParseDouble(sentence.body[i + 5], msg->info[index].adr);
    valid &= ParseFloat(sentence.body[i + 6], msg->info[index].adr_std);
    valid &= ParseFloat(sentence.body[i + 7], msg->info[index].dopp);
    valid &=
        ParseFloat(sentence.body[i + 8], msg->info[index].noise_density_ratio);
    valid &= ParseFloat(sentence.body[i + 9], msg->info[index].locktime);
    std::string track = "0x" + sentence.body[i + 10]; // This number is in hex
    valid &= ParseUInt32(track, msg->info[index].tracking_status, 16);
  }
  if (!valid) {
    throw ParseException("Error parsing RANGE log.");
  }
  return msg;
}
