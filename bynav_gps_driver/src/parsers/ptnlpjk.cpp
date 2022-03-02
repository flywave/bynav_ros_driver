#include <bynav_gps_driver/parsers/ptnlpjk.h>

#include <bynav_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace bynav_gps_driver {

const std::string PtnlPJKParser::MESSAGE_NAME = "PTNLPJK";

uint32_t PtnlPJKParser::GetMessageId() const { return MESSAGE_ID; }

const std::string PtnlPJKParser::GetMessageName() const { return MESSAGE_NAME; }

bynav_gps_msgs::PtnlPJKPtr
PtnlPJKParser::ParseAscii(const BynavSentence &sentence) noexcept(false) {
  bynav_gps_msgs::PtnlPJKPtr msg =
      boost::make_shared<bynav_gps_msgs::BynavPJK>();
  HeaderParser h_parser;
  msg->bynav_msg_header = h_parser.ParseAscii(sentence);

  if (sentence.body.size() != ASCII_LENGTH) {
    std::stringstream error;
    error << "Unexpected number of PTNLPJK message fields: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  msg->solution_status = sentence.body[0];
  msg->position_type = sentence.body[1];

  valid = valid && ParseDouble(sentence.body[2], msg->x);
  valid = valid && ParseDouble(sentence.body[3], msg->y);
  valid = valid && ParseDouble(sentence.body[4], msg->z);

  valid = valid && ParseFloat(sentence.body[5], msg->x_sigma);
  valid = valid && ParseFloat(sentence.body[6], msg->y_sigma);
  valid = valid && ParseFloat(sentence.body[7], msg->z_sigma);

  msg->velocity_solution_status = sentence.body[8];
  msg->velocity_type = sentence.body[9];

  valid = valid && ParseDouble(sentence.body[10], msg->x_vel);
  valid = valid && ParseDouble(sentence.body[11], msg->y_vel);
  valid = valid && ParseDouble(sentence.body[12], msg->z_vel);

  valid = valid && ParseFloat(sentence.body[13], msg->x_vel_sigma);
  valid = valid && ParseFloat(sentence.body[14], msg->y_vel_sigma);
  valid = valid && ParseFloat(sentence.body[15], msg->z_vel_sigma);

  msg->base_station_id = sentence.body[16];
  valid = valid && ParseFloat(sentence.body[17], msg->velocity_latency);

  valid = valid && ParseFloat(sentence.body[18], msg->diff_age);
  valid = valid && ParseFloat(sentence.body[19], msg->solution_age);
  valid = valid && ParseUInt8(sentence.body[20], msg->num_satellites_tracked);
  valid = valid &&
          ParseUInt8(sentence.body[21], msg->num_satellites_used_in_solution);
  valid = valid && ParseUInt8(sentence.body[22],
                              msg->num_gps_and_glonass_l1_used_in_solution);
  valid =
      valid && ParseUInt8(sentence.body[23],
                          msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

  uint32_t extended_solution_status = 0;
  valid = valid && ParseUInt32(sentence.body[25], extended_solution_status, 16);
  GetExtendedSolutionStatusMessage(extended_solution_status,
                                   msg->extended_solution_status);

  uint32_t signal_mask = 0;
  valid = valid && ParseUInt32(sentence.body[27], signal_mask, 16);
  GetSignalsUsed(signal_mask, msg->signal_mask);

  if (!valid) {
    throw ParseException("Invalid field in PTNLPJK message");
  }

  return msg;
}
} // namespace bynav_gps_driver
