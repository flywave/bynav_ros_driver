#include <bynav_gps_driver/parsers/ptnlavr.h>

#include <bynav_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace bynav_gps_driver {

const std::string PtnlAvrParser::MESSAGE_NAME = "PTNLAVR";

uint32_t PtnlAvrParser::GetMessageId() const { return 0; }

const std::string PtnlAvrParser::GetMessageName() const { return MESSAGE_NAME; }

bynav_gps_msgs::PtnlAvrPtr
PtnlAvrParser::ParseAscii(const NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 13;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected PTNLAVR length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::PtnlAvrPtr msg =
      boost::make_shared<bynav_gps_msgs::PtnlAvr>();

  msg->message_id = "PTNLAVR";

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[2], msg->utc_seconds);

  valid = valid && ParseDouble(sentence.body[3], msg->yaw);
  valid = valid && ParseDouble(sentence.body[5], msg->tilt);

  valid = valid && ParseFloat(sentence.body[9], msg->baseline_length);

  valid = valid && ParseUInt32(sentence.body[10], msg->solution_status);
  valid = valid && ParseFloat(sentence.body[11], msg->pdop);
  valid = valid &&
          ParseUInt8(sentence.body[12], msg->num_satellites_used_in_solution);

  if (!valid) {
    throw ParseException("Invalid field in PTNLAVR message");
  }

  return msg;
}
} // namespace bynav_gps_driver
