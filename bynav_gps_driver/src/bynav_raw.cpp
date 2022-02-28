#include <bynav_gps_driver/bynav_raw.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sstream>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

namespace bynav_gps_driver {

BynavRaw::BynavRaw() {}

bool BynavRaw::Connect(const std::string &device, ConnectionType connection) {
  BynavMessageOpts opts;
  opts["RANGECMPB"] = -1;
  opts["GLOEPHEMERISB"] = -1;
  opts["BDSEPHEMERISB"] = -1;
  opts["RAWEPHEMB"] = -1;
  opts["IONUTCB"] = -1;
  opts["BDSALMANACB"] = -1;
  return BynavConnection::Connect(device, connection, opts);
}

void BynavRaw::GetRawMessages(
    std::vector<bynav_gps_msgs::RawPtr> &raw_messages) {
  raw_msgs_.clear();
  raw_msgs_.insert(raw_msgs_.end(), raw_messages.begin(), raw_messages.end());
  raw_messages.clear();
}

BynavRaw::ReadResult BynavRaw::ProcessData() {
  BynavRaw::ReadResult read_result = ReadData();

  if (read_result != READ_SUCCESS) {
    return read_result;
  }

  ros::Time stamp = ros::Time::now();
  std::vector<NmeaSentence> nmea_sentences;
  std::vector<BynavSentence> bynav_sentences;
  std::vector<BinaryMessage> binary_messages;

  if (!data_buffer_.empty()) {
    raw_buffer_.insert(raw_buffer_.end(), data_buffer_.begin(),
                       data_buffer_.end());
    data_buffer_.clear();

    std::string remaining_buffer;

    if (!extractor_.ExtractCompleteMessages(raw_buffer_, nmea_sentences,
                                            bynav_sentences, binary_messages,
                                            remaining_buffer)) {
      read_result = READ_PARSE_FAILED;
      error_msg_ = "Parse failure extracting sentences.";
    }

    nmea_buffer_ = remaining_buffer;

    ROS_DEBUG("Parsed: %lu NMEA / %lu Bynav / %lu Binary messages",
              nmea_sentences.size(), bynav_sentences.size(),
              binary_messages.size());
    if (!nmea_buffer_.empty()) {
      ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
    }
  }

  for (auto &msg : binary_messages) {
    auto si = static_cast<int32_t>(msg.header_.header_length_ +
                                   msg.header_.message_length_ + 4);
    bynav_gps_msgs::RawPtr ros_msg = boost::make_shared<bynav_gps_msgs::Raw>();
    ros_msg->header.stamp = stamp;
    ros_msg->data.resize(si);
    msg.header_.WriteHeader(ros_msg->data.data());
    std::copy(msg.data_.begin(), msg.data_.end(),
              (ros_msg->data.data() + msg.header_.header_length_));
    std::copy(reinterpret_cast<uint8_t *>(&msg.crc),
              (reinterpret_cast<uint8_t *>(&msg.crc) + 4),
              (ros_msg->data.data() + msg.header_.header_length_ +
               msg.header_.message_length_));
    raw_msgs_.push_back(std::move(ros_msg));
  }
}

} // namespace bynav_gps_driver
