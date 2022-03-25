#include <bynav_gps_driver/parsers/heading.h>

#include <bynav_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace bynav_gps_driver {

const std::string HeadingParser::MESSAGE_NAME = "HEADING";

uint32_t HeadingParser::GetMessageId() const { return MESSAGE_ID; }

const std::string HeadingParser::GetMessageName() const { return MESSAGE_NAME; }

bynav_gps_msgs::HeadingPtr
HeadingParser::ParseBinary(const BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected HEADING message length: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::HeadingPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::Heading>();
  HeaderParser header_parser;
  ros_msg->bynav_msg_header = header_parser.ParseBinary(bin_msg);
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
  ros_msg->position_type = POSITION_TYPES[pos_type];

  ros_msg->baseline_length = ParseFloat(&bin_msg.data_[8]);

  ros_msg->heading = ParseFloat(&bin_msg.data_[12]);
  ros_msg->pitch = ParseFloat(&bin_msg.data_[16]);

  ros_msg->heading_sigma = ParseFloat(&bin_msg.data_[24]);
  ros_msg->pitch_sigma = ParseFloat(&bin_msg.data_[28]);

  ros_msg->rover_station_id.resize(4);
  std::copy(&bin_msg.data_[32], &bin_msg.data_[36],
            &ros_msg->rover_station_id[0]);

  ros_msg->master_station_id.resize(4);
  std::copy(&bin_msg.data_[36], &bin_msg.data_[40],
            &ros_msg->master_station_id[0]);

  ros_msg->num_satellites_tracked = bin_msg.data_[40];
  ros_msg->num_satellites_used_in_solution = bin_msg.data_[41];
  ros_msg->num_satellites_above_elevation_mask_angle = bin_msg.data_[42];
  ros_msg->num_satellites_above_elevation_mask_angle_l2 = bin_msg.data_[43];

  ros_msg->solution_source = SolutionSourceToMsgEnum(bin_msg.data_[44]);

  GetExtendedSolutionStatusMessage(bin_msg.data_[45],
                                   ros_msg->extended_solution_status);

  GetSignalsUsed(bin_msg.data_[47], ros_msg->signal_mask);

  return ros_msg;
}

bynav_gps_msgs::HeadingPtr
HeadingParser::ParseAscii(const BynavSentence &sentence) {
  bynav_gps_msgs::HeadingPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::Heading>();

  HeaderParser h_parser;
  ros_msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  if (sentence.body.size() != ASCII_LENGTH) {
    std::stringstream error;
    error << "Unexpected number of HEADING message fields: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  ros_msg->solution_status = sentence.body[0];
  ros_msg->position_type = sentence.body[1];

  valid = valid && ParseFloat(sentence.body[2], ros_msg->baseline_length);

  valid = valid && ParseFloat(sentence.body[3], ros_msg->heading);
  valid = valid && ParseFloat(sentence.body[4], ros_msg->pitch);

  valid = valid && ParseFloat(sentence.body[6], ros_msg->heading_sigma);
  valid = valid && ParseFloat(sentence.body[7], ros_msg->pitch_sigma);

  ros_msg->rover_station_id = sentence.body[8];

  ros_msg->master_station_id = sentence.body[9];

  valid =
      valid && ParseUInt8(sentence.body[10], ros_msg->num_satellites_tracked);
  valid = valid && ParseUInt8(sentence.body[11],
                              ros_msg->num_satellites_used_in_solution);
  valid =
      valid && ParseUInt8(sentence.body[12],
                          ros_msg->num_satellites_above_elevation_mask_angle);
  valid = valid &&
          ParseUInt8(sentence.body[13],
                     ros_msg->num_satellites_above_elevation_mask_angle_l2);

  uint32_t solution_source = 0;
  valid = valid && ParseUInt32(sentence.body[14], solution_source, 16);
  ros_msg->solution_source = SolutionSourceToMsgEnum((uint8_t)solution_source);

  uint32_t extended_solution_status = 0;
  valid = valid && ParseUInt32(sentence.body[15], extended_solution_status, 16);
  GetExtendedSolutionStatusMessage(extended_solution_status,
                                   ros_msg->extended_solution_status);

  uint32_t signal_mask = 0;
  valid = valid && ParseUInt32(sentence.body[17], signal_mask, 16);
  GetSignalsUsed(signal_mask, ros_msg->signal_mask);

  if (!valid) {
    throw ParseException("Invalid field in HEADING message");
  }

  return ros_msg;
}

uint8_t HeadingParser::SolutionSourceToMsgEnum(uint8_t source_mask) {
  uint8_t source_bits = (source_mask & 0x0Cu) >> 2u;
  switch (source_bits) {
  case 0:
    return bynav_gps_msgs::Heading::SOURCE_PRIMARY_ANTENNA;
  case 1:
    return bynav_gps_msgs::Heading::SOURCE_SECONDARY_ANTENNA;
  default:
    throw ParseException(
        "HEADING Solution Source could not be parsed due to unknown source");
  }
}
} // namespace bynav_gps_driver
