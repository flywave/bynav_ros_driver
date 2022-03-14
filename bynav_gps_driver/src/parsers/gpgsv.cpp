#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpgsv.h>

const std::string bynav_gps_driver::GpgsvParser::MESSAGE_NAME = "GPGSV";

uint32_t bynav_gps_driver::GpgsvParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpgsvParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpgsvPtr bynav_gps_driver::GpgsvParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) {
  const size_t MIN_LENGTH = 4;
  if (sentence.body.size() < MIN_LENGTH) {
    std::stringstream error;
    error << "Expected GPGSV length >= " << MIN_LENGTH
          << ", actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::GpgsvPtr msg = boost::make_shared<bynav_gps_msgs::Gpgsv>();
  msg->message_id = sentence.body[0];
  if (!ParseUInt8(sentence.body[1], msg->n_msgs)) {
    throw new ParseException("Error parsing n_msgs in GPGSV.");
  }
  if (msg->n_msgs > 9) {
    std::stringstream error;
    error << "n_msgs in GPGSV was too large (" << msg->n_msgs << ").";
    throw ParseException(error.str());
  }

  if (!ParseUInt8(sentence.body[2], msg->msg_number)) {
    throw ParseException("Error parsing msg_number in GPGSV.");
  }
  if (msg->msg_number > msg->n_msgs) {
    std::stringstream error;
    error << "msg_number in GPGSV was larger than n_msgs (" << msg->msg_number
          << " > " << msg->n_msgs << ").";
    throw ParseException(error.str());
  }
  if (!ParseUInt8(sentence.body[3], msg->n_satellites)) {
    throw ParseException("Error parsing n_satellites in GPGSV.");
  }

  size_t n_sats_in_sentence = 4;
  if (msg->msg_number == msg->n_msgs) {
    n_sats_in_sentence = msg->n_satellites % static_cast<uint8_t>(4);
  }
  size_t expected_length = MIN_LENGTH + 4 * n_sats_in_sentence;
  if (n_sats_in_sentence == 0) {
    expected_length += 4;
  }
  if (sentence.body.size() != expected_length &&
      sentence.body.size() != expected_length - 1) {
    std::stringstream ss;
    for (size_t i = 0; i < sentence.body.size(); ++i) {
      ss << sentence.body[i];
      if ((i + 1) < sentence.body.size()) {
        ss << ",";
      }
    }
    std::stringstream error;
    error << "Expected GPGSV length = " << expected_length
          << " for message with " << n_sats_in_sentence
          << " satellites, actual length = " << sentence.body.size() << "\n"
          << ss.str().c_str();
    throw ParseException(error.str());
  }
  msg->satellites.resize(n_sats_in_sentence);
  for (size_t sat = 0, index = MIN_LENGTH; sat < n_sats_in_sentence;
       ++sat, index += 4) {
    if (!ParseUInt8(sentence.body[index], msg->satellites[sat].prn)) {
      std::stringstream error;
      error << "Error parsing prn for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }

    float elevation;
    if (!ParseFloat(sentence.body[index + 1], elevation)) {
      std::stringstream error;
      error << "Error parsing elevation for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }
    msg->satellites[sat].elevation = static_cast<uint8_t>(elevation);

    float azimuth;
    if (!ParseFloat(sentence.body[index + 2], azimuth)) {
      std::stringstream error;
      error << "Error parsing azimuth for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }
    msg->satellites[sat].azimuth = static_cast<uint16_t>(azimuth);

    if ((index + 3) >= sentence.body.size() ||
        sentence.body[index + 3].empty()) {
      msg->satellites[sat].snr = -1;
    } else {
      uint8_t snr;
      if (!ParseUInt8(sentence.body[index + 3], snr)) {
        std::stringstream error;
        error << "Error parsing snr for satellite " << sat << " in GPGSV.";
        throw ParseException(error.str());
      }

      msg->satellites[sat].snr = static_cast<int8_t>(snr);
    }
  }
  return msg;
}
