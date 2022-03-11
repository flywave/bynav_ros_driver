#ifndef BYNAV_MESSAGE_EXTRACTOR_H_
#define BYNAV_MESSAGE_EXTRACTOR_H_

#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <bynav_gps_msgs/BynavCorrectedImuData.h>
#include <bynav_gps_msgs/BynavMessageHeader.h>
#include <bynav_gps_msgs/BynavPosition.h>
#include <bynav_gps_msgs/BynavReceiverStatus.h>
#include <bynav_gps_msgs/Gpgga.h>
#include <bynav_gps_msgs/Gpgsv.h>
#include <bynav_gps_msgs/Gprmc.h>
#include <bynav_gps_msgs/Psrvel.h>
#include <bynav_gps_msgs/Time.h>
#include <gps_common/GPSFix.h>

#include <bynav_gps_driver/binary_message.h>
#include <bynav_gps_driver/bynav_sentence.h>
#include <bynav_gps_driver/nmea_sentence.h>
#include <bynav_gps_driver/rtcm_sentence.h>

namespace bynav_gps_driver {

class BynavMessageExtractor {
public:
  bool ExtractCompleteMessages(const std::string &input,
                               std::vector<NmeaSentence> &nmea_sentences,
                               std::vector<BynavSentence> &bynav_sentences,
                               std::vector<BinaryMessage> &binary_messages,
                               std::string &remaining,
                               bool keep_nmea_container = false);

  double GetMostRecentUtcTime(const std::vector<NmeaSentence> &sentences);

  void GetGpsFixMessage(const bynav_gps_msgs::Gprmc &gprmc,
                        const bynav_gps_msgs::Gpgga &gpgga,
                        const gps_common::GPSFixPtr &gps_fix);

private:
  static const std::string CHECKSUM_FLAG;
  static const std::string FIELD_SEPARATOR;
  static const std::string HEADER_SEPARATOR;
  static const std::string NMEA_SENTENCE_FLAG;
  static const std::string BYNAV_SENTENCE_FLAG;
  static const std::string BYNAV_ASCII_FLAGS;
  static const std::string BYNAV_MM_FLAG;
  static const std::string BYNAV_BINARY_SYNC_BYTES;
  static const std::string BYNAV_ENDLINE;

  static constexpr uint32_t BYNAV_CRC32_POLYNOMIAL = 0xEDB88320L;

  uint32_t CalculateBlockCRC32(uint32_t ulCount, const uint8_t *ucBuffer);

  uint32_t CRC32Value(int32_t i);

  void FindAsciiSentence(const std::string &sentence, size_t current_idx,
                         size_t &start_idx, size_t &end_idx,
                         size_t &invalid_char_idx);

  int32_t GetBinaryMessage(const std::string &str, size_t start_idx,
                           BinaryMessage &msg);

  bool GetBynavMessageParts(const std::string &sentence,
                            std::string &message_id,
                            std::vector<std::string> &header,
                            std::vector<std::string> &body);

  int32_t GetNmeaSentence(const std::string &str, size_t start_idx,
                          std::string &sentence, bool keep_container = false);

  int32_t GetBynavSentence(const std::string &str, size_t start_idx,
                           std::string &sentence);

  size_t GetSentenceChecksumStart(const std::string &str, size_t start_idx);

  uint8_t NmeaChecksum(const std::string &sentence);

  bool VectorizeBynavSentence(const std::string &data, BynavSentence &sentence);

  void VectorizeNmeaSentence(const std::string &sentence,
                             NmeaSentence &vectorized_message);

  void VectorizeString(const std::string &str,
                       std::vector<std::string> &vectorized_message,
                       const std::string &delimiters);
};
} // namespace bynav_gps_driver

#endif // BYNAV_MESSAGE_EXTRACTOR_H_
