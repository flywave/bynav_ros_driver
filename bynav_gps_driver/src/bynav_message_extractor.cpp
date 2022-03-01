#include <bynav_gps_driver/bynav_message_extractor.h>

#include <limits>
#include <sstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <ros/ros.h>

#include <bynav_gps_driver/parsers/gpgga.h>
#include <bynav_gps_driver/parsers/gprmc.h>
#include <bynav_gps_driver/parsers/header.h>
#include <swri_string_util/string_util.h>

namespace bynav_gps_driver {

const std::string BynavMessageExtractor::CHECKSUM_FLAG = "*";
const std::string BynavMessageExtractor::FIELD_SEPARATOR = ",";
const std::string BynavMessageExtractor::HEADER_SEPARATOR = ";";
const std::string BynavMessageExtractor::NMEA_SENTENCE_FLAG = "$";
const std::string BynavMessageExtractor::BYNAV_SENTENCE_FLAG = "#";
const std::string BynavMessageExtractor::BYNAV_MM_FLAG = "%";
const std::string BynavMessageExtractor::BYNAV_ASCII_FLAGS = "$#%";
const std::string BynavMessageExtractor::BYNAV_BINARY_SYNC_BYTES =
    "\xAA\x44\x12";
const std::string BynavMessageExtractor::BYNAV_ENDLINE = "\r\n";

uint32_t BynavMessageExtractor::CRC32Value(int32_t i) {
  int32_t j;
  uint32_t ulCRC;
  ulCRC = static_cast<uint32_t>(i);
  for (j = 8; j > 0; j--) {
    if (ulCRC & 1u)
      ulCRC = static_cast<uint32_t>((ulCRC >> 1u) ^ BYNAV_CRC32_POLYNOMIAL);
    else
      ulCRC >>= 1u;
  }
  return ulCRC;
}

uint32_t BynavMessageExtractor::CalculateBlockCRC32(uint32_t ulCount,
                                                    const uint8_t *ucBuffer) {
  uint32_t ulTemp1;
  uint32_t ulTemp2;
  uint32_t ulCRC = 0;
  while (ulCount-- != 0) {
    ulTemp1 = static_cast<uint32_t>((ulCRC >> 8u) & 0x00FFFFFFL);
    ulTemp2 = CRC32Value(((int32_t)ulCRC ^ *ucBuffer++) & 0xffu);
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return (ulCRC);
}

uint8_t BynavMessageExtractor::NmeaChecksum(const std::string &sentence) {
  uint8_t checksum = 0;
  std::string::const_iterator it = sentence.begin();
  for (; it != sentence.end(); ++it) {
    checksum ^= *it;
  }
  return checksum;
}

size_t BynavMessageExtractor::GetSentenceChecksumStart(const std::string &str,
                                                       size_t start_idx) {
  return str.find(CHECKSUM_FLAG, start_idx);
}

void BynavMessageExtractor::VectorizeString(
    const std::string &str, std::vector<std::string> &vectorized_message,
    const std::string &delimiters) {
  boost::algorithm::split(vectorized_message, str,
                          boost::algorithm::is_any_of(delimiters));
}

bool BynavMessageExtractor::GetBynavMessageParts(
    const std::string &sentence, std::string &message_id,
    std::vector<std::string> &header, std::vector<std::string> &body) {
  message_id.clear();
  header.clear();
  body.clear();

  std::vector<std::string> vectorized_message;
  VectorizeString(sentence, vectorized_message, HEADER_SEPARATOR);

  if (vectorized_message.size() != 2) {
    return false;
  }

  VectorizeString(vectorized_message[0], header, FIELD_SEPARATOR);

  VectorizeString(vectorized_message[1], body, FIELD_SEPARATOR);

  if (!header.empty()) {
    message_id = header.front();
  } else {
    return false;
  }
  return true;
}

int32_t BynavMessageExtractor::GetBinaryMessage(const std::string &str,
                                                size_t start_idx,
                                                BinaryMessage &msg) {
  if (str.length() < HeaderParser::BINARY_HEADER_LENGTH + 4) {
    ROS_DEBUG("Binary message was too short to parse.");
    return -1;
  }

  ROS_DEBUG("Reading binary header.");
  msg.header_.ParseHeader(reinterpret_cast<const uint8_t *>(&str[start_idx]));
  auto data_start =
      static_cast<uint16_t>(msg.header_.header_length_ + start_idx);
  uint16_t data_length = msg.header_.message_length_;

  if (msg.header_.sync0_ != static_cast<uint8_t>(BYNAV_BINARY_SYNC_BYTES[0]) ||
      msg.header_.sync1_ != static_cast<uint8_t>(BYNAV_BINARY_SYNC_BYTES[1]) ||
      msg.header_.sync2_ != static_cast<uint8_t>(BYNAV_BINARY_SYNC_BYTES[2])) {
    ROS_ERROR("Sync bytes were incorrect; this should never happen and is "
              "definitely a bug: %x %x %x",
              msg.header_.sync0_, msg.header_.sync1_, msg.header_.sync2_);
    return -2;
  }

  if (msg.header_.header_length_ != HeaderParser::BINARY_HEADER_LENGTH) {
    ROS_WARN("Binary header length was unexpected: %u (expected %u)",
             msg.header_.header_length_, HeaderParser::BINARY_HEADER_LENGTH);
  }

  ROS_DEBUG("Msg ID: %u    Data start / length: %u / %u",
            msg.header_.message_id_, data_start, data_length);

  if (data_start + data_length + 4 > str.length()) {
    ROS_DEBUG("Not enough data.");
    return -1;
  }

  ROS_DEBUG("Reading binary message data.");
  msg.data_.resize(data_length);
  std::copy(&str[data_start], &str[data_start + data_length],
            reinterpret_cast<char *>(&msg.data_[0]));

  ROS_DEBUG("Calculating CRC.");

  uint32_t crc = CalculateBlockCRC32(
      static_cast<uint32_t>(msg.header_.header_length_) + data_length,
      reinterpret_cast<const uint8_t *>(&str[start_idx]));

  ROS_DEBUG("Reading CRC.");
  msg.crc_ = ParseUInt32(
      reinterpret_cast<const uint8_t *>(&str[data_start + data_length]));

  if (crc != msg.crc_) {
    ROS_DEBUG("Invalid CRC;  Calc: %u    In msg: %u", crc, msg.crc_);
    return -2;
  }

  ROS_DEBUG("Finishing reading binary message.");
  return static_cast<int32_t>(msg.header_.header_length_ + data_length + 4);
}

int32_t BynavMessageExtractor::GetBynavSentence(const std::string &str,
                                                size_t start_idx,
                                                std::string &sentence) {
  sentence.clear();

  size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
  if (checksum_start == std::string::npos) {
    return -1;
  } else if (checksum_start + 8 >= str.size()) {
    return -1;
  } else {
    sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
    std::string checksum_str = str.substr(checksum_start + 1, 8);
    uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
    uint64_t calculated_checksum = CalculateBlockCRC32(
        static_cast<uint32_t>(sentence.size()),
        reinterpret_cast<const uint8_t *>(sentence.c_str()));

    if (checksum == ULONG_MAX) {
      return 1;
    } else if (static_cast<uint32_t>(checksum) == calculated_checksum) {
      return 0;
    } else {
      ROS_WARN("Expected checksum: [%lx]", calculated_checksum);
      return 1;
    }
  }
}

int32_t BynavMessageExtractor::GetNmeaSentence(const std::string &str,
                                               size_t start_idx,
                                               std::string &sentence,
                                               bool keep_container) {
  sentence.clear();

  size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
  if (checksum_start == std::string::npos) {
    return -1;
  } else if (checksum_start + 2 >= str.size()) {
    return -1;
  } else {
    sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
    std::string checksum_str = str.substr(checksum_start + 1, 2);
    uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
    uint64_t calculated_checksum = NmeaChecksum(sentence);
    if (checksum == ULONG_MAX) {
      return 1;
    } else if (static_cast<uint32_t>(checksum) == calculated_checksum) {
      if (keep_container) {
        sentence.insert(0, "$");
        std::string recreated_checksum_str("*");
        recreated_checksum_str += checksum_str;
        sentence.insert(sentence.end(), recreated_checksum_str.begin(),
                        recreated_checksum_str.end());
      }
      return 0;
    } else {
      ROS_WARN("Expected: [%lx]", calculated_checksum);
      return 1;
    }
  }
}

void BynavMessageExtractor::FindAsciiSentence(const std::string &sentence,
                                              size_t current_idx,
                                              size_t &start_idx,
                                              size_t &end_idx,
                                              size_t &invalid_char_idx) {
  start_idx = sentence.find_first_of(BYNAV_ASCII_FLAGS, current_idx);
  end_idx = std::string::npos;
  invalid_char_idx = std::string::npos;

  if (start_idx == std::string::npos) {
    return;
  }

  end_idx = sentence.find(BYNAV_ENDLINE, start_idx);

  size_t search_stop_idx = std::min(end_idx, sentence.length());
  for (size_t i = start_idx; i < search_stop_idx; i++) {
    if (sentence[i] == 9 || sentence[i] == 10 || sentence[i] == 11 ||
        sentence[i] == 13 || (sentence[i] >= 32 && sentence[i] <= 126)) {
      continue;
    }

    invalid_char_idx = i;
    break;
  }
}

bool BynavMessageExtractor::VectorizeBynavSentence(const std::string &data,
                                                   BynavSentence &sentence) {
  return GetBynavMessageParts(data, sentence.id, sentence.header,
                              sentence.body);
}

void BynavMessageExtractor::VectorizeNmeaSentence(
    const std::string &sentence, NmeaSentence &vectorized_message) {
  VectorizeString(sentence, vectorized_message.body, FIELD_SEPARATOR);
  if (!vectorized_message.body.empty()) {
    vectorized_message.id = vectorized_message.body.front();
  }
}

bool BynavMessageExtractor::ExtractCompleteMessages(
    const std::string &input, std::vector<NmeaSentence> &nmea_sentences,
    std::vector<BynavSentence> &bynav_sentences,
    std::vector<BinaryMessage> &binary_messages, std::string &remaining,
    bool keep_nmea_container) {
  bool parse_error = false;

  size_t sentence_start = 0;
  while (sentence_start != std::string::npos && sentence_start < input.size()) {
    size_t ascii_start_idx;
    size_t ascii_end_idx;
    size_t invalid_ascii_idx;
    size_t binary_start_idx =
        input.find(BYNAV_BINARY_SYNC_BYTES, sentence_start);

    FindAsciiSentence(input, sentence_start, ascii_start_idx, ascii_end_idx,
                      invalid_ascii_idx);

    ROS_DEBUG(
        "Binary start: %lu   ASCII start / end / invalid: %lu / %lu / %lu",
        binary_start_idx, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);

    if (binary_start_idx == std::string::npos &&
        ascii_start_idx == std::string::npos) {
      break;
    }

    if (ascii_start_idx == std::string::npos ||
        (binary_start_idx != std::string::npos &&
         binary_start_idx < ascii_start_idx)) {
      BinaryMessage cur_msg;
      int32_t result = GetBinaryMessage(input, binary_start_idx, cur_msg);
      if (result > 0) {
        binary_messages.push_back(cur_msg);
        sentence_start += binary_start_idx + result;
        ROS_DEBUG("Parsed a binary message with %u bytes.", result);
      } else if (result == -1) {
        remaining = input.substr(binary_start_idx);
        ROS_DEBUG("Binary message was incomplete, waiting for more.");
        break;
      } else {
        sentence_start += 1;
        ROS_WARN("Invalid binary message checksum");
        parse_error = true;
      }
    } else {
      size_t ascii_len = ascii_end_idx - ascii_start_idx;
      if (invalid_ascii_idx != std::string::npos) {
        ROS_WARN("Invalid ASCII char: [%s]",
                 input.substr(ascii_start_idx, ascii_len).c_str());
        ROS_WARN(
            "                     %s^",
            std::string(invalid_ascii_idx - ascii_start_idx - 1, ' ').c_str());
        sentence_start += invalid_ascii_idx + 1;
      } else if (ascii_end_idx != std::string::npos) {
        ROS_DEBUG("ASCII sentence:\n[%s]",
                  input.substr(ascii_start_idx, ascii_len).c_str());
        if (input[ascii_start_idx] == NMEA_SENTENCE_FLAG[0]) {
          std::string cur_sentence;
          int32_t result = GetNmeaSentence(input, ascii_start_idx, cur_sentence,
                                           keep_nmea_container);
          if (result == 0) {
            nmea_sentences.emplace_back(NmeaSentence());
            VectorizeNmeaSentence(cur_sentence, nmea_sentences.back());
            sentence_start = ascii_end_idx;
          } else if (result < 0) {
            remaining = input.substr(ascii_start_idx);
            ROS_DEBUG("Waiting for more NMEA data.");
            break;
          } else {
            ROS_WARN("Invalid NMEA checksum for: [%s]",
                     input.substr(ascii_start_idx, ascii_len).c_str());
            sentence_start += 1;
            parse_error = true;
          }
        } else if (input[ascii_start_idx] == BYNAV_SENTENCE_FLAG[0]) {
          std::string cur_sentence;
          int32_t result =
              GetBynavSentence(input, ascii_start_idx, cur_sentence);
          if (result == 0) {
            bynav_sentences.emplace_back(BynavSentence());
            if (!VectorizeBynavSentence(cur_sentence, bynav_sentences.back())) {
              bynav_sentences.pop_back();
              parse_error = true;
              ROS_ERROR_THROTTLE(1.0, "Unable to vectorize bynav sentence");
            }
            sentence_start = ascii_end_idx;
          } else if (result < 0) {
            remaining = input.substr(ascii_start_idx);
            ROS_DEBUG("Waiting for more Bynav data.");
            break;
          } else {
            sentence_start += 1;
            ROS_WARN("Invalid Bynav checksum for: [%s]",
                     input.substr(ascii_start_idx, ascii_len).c_str());
            parse_error = true;
          }
        } else if (input[ascii_start_idx] == BYNAV_MM_FLAG[0]) {
          //TODO
        }
      } else {
        ROS_DEBUG("Incomplete ASCII sentence, waiting for more.");
        remaining = input.substr(ascii_start_idx);
        break;
      }
    }
  }

  return !parse_error;
}

double BynavMessageExtractor::GetMostRecentUtcTime(
    const std::vector<NmeaSentence> &sentences) {
  std::vector<NmeaSentence>::const_reverse_iterator iter;
  for (iter = sentences.rbegin(); iter != sentences.rend(); iter++) {
    if (iter->id == GpggaParser::MESSAGE_NAME ||
        iter->id == GprmcParser::MESSAGE_NAME) {
      if (iter->body.size() > 1) {
        if (iter->body[1].empty() || iter->body[1] == "0") {
          return 0;
        } else {
          double utc_float;
          if (swri_string_util::ToDouble(iter->body[1], utc_float)) {
            return UtcFloatToSeconds(utc_float);
          }
          return 0;
        }
      }
    }
  }

  return 0;
}

void BynavMessageExtractor::GetGpsFixMessage(
    const bynav_gps_msgs::Gprmc &gprmc, const bynav_gps_msgs::Gpgga &gpgga,
    const gps_common::GPSFixPtr &gps_fix) {
  gps_fix->header.stamp = gpgga.header.stamp;
  gps_fix->altitude = gpgga.alt;
  gps_fix->hdop = gpgga.hdop;
  gps_fix->latitude = gprmc.lat;
  if (gpgga.lat_dir == "S") {
    gps_fix->latitude *= -1;
  }

  gps_fix->longitude = gprmc.lon;
  if (gpgga.lon_dir == "W") {
    gps_fix->longitude *= -1;
  }
  gps_fix->speed = gprmc.speed;
  gps_fix->time = gpgga.utc_seconds;
  gps_fix->track = gprmc.track;

  switch (gpgga.gps_qual) {
  case bynav_gps_msgs::Gpgga::GPS_QUAL_INVALID:
    gps_fix->status.status = gps_common::GPSStatus::STATUS_NO_FIX;
    break;
  case bynav_gps_msgs::Gpgga::GPS_QUAL_WASS:
    gps_fix->status.status = gps_common::GPSStatus::STATUS_WAAS_FIX;
    break;
  default:
    gps_fix->status.status = gps_common::GPSStatus::STATUS_FIX;
    break;
  }
  gps_fix->status.satellites_used = static_cast<uint16_t>(gpgga.num_sats);
}

} // namespace bynav_gps_driver
