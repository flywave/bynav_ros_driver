#include <bynav_gps_driver/parsers/ptnlpjk.h>

#include <bynav_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace bynav_gps_driver {

const std::string PtnlPJKParser::MESSAGE_NAME = "PTNLPJK";

uint32_t PtnlPJKParser::GetMessageId() const { return 0; }

const std::string PtnlPJKParser::GetMessageName() const { return MESSAGE_NAME; }

bynav_gps_msgs::PtnlPJKPtr
PtnlPJKParser::ParseAscii(const NmeaSentence &sentence) {
  const size_t EXPECTED_LEN = 13;

  if (sentence.body.size() < EXPECTED_LEN) {
    std::stringstream error;
    error << "Expected PTNLPJK length = " << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::PtnlPJKPtr msg =
      boost::make_shared<bynav_gps_msgs::PtnlPJK>();
      
  msg->message_id = "PTNLPJK";

  bool valid = true;

  valid = valid && ParseDouble(sentence.body[2], msg->utc_seconds);

  char parse[3][20] = {{0}, {0}};
  int len = sscanf(sentence.body[3].data(), "%2s%2s%2s", parse[0], parse[1],
                   parse[2]);
  if (len == 3) {
    msg->utc_month = atoi(parse[0]);
    msg->utc_day = atoi(parse[1]);
    msg->utc_year = atoi(parse[2]) + 2000;
  }

  valid = valid && ParseDouble(sentence.body[4], msg->x);
  msg->x_dir = sentence.body[5];
  valid = valid && ParseDouble(sentence.body[6], msg->y);
  msg->y_dir = sentence.body[7];

  valid = valid && ParseUInt32(sentence.body[8], msg->solution_status);
  valid = valid &&
          ParseUInt8(sentence.body[9], msg->num_satellites_used_in_solution);
  valid = valid && ParseDouble(sentence.body[10], msg->hdop);
  if (sentence.body[11].size() > 3) {
    msg->datum_id = sentence.body[11].substr(0, 3);
    if (msg->datum_id == "EHT" || msg->datum_id == "GHT") {
      valid =
          valid && ParseDouble(sentence.body[9].substr(3), msg->height_offset);
    }
  }
  msg->height_unit = sentence.body[12];
  if (!valid) {
    throw ParseException("Invalid field in PTNLPJK message");
  }

  return msg;
}
} // namespace bynav_gps_driver
