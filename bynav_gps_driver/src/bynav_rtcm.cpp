#include <bynav_gps_driver/bynav_rtcm.h>
#include <bynav_gps_driver/parsers/rtcm.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sstream>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

namespace bynav_gps_driver {

BynavRtcm::BynavRtcm() {}

bool BynavRtcm::Connect(const std::string &device, ConnectionType connection) {
  BynavMessageOpts opts;
  opts["rtcm1074"] = 1;
  opts["rtcm1084"] = 1;
  opts["rtcm1094"] = 1;
  opts["rtcm1114"] = 1;
  opts["rtcm1124"] = 1;
  opts["rtcm1134"] = 1;
  opts["rtcm1006"] = 5;
  opts["rtcm1033"] = 10;
  return BynavConnection::Connect(device, connection, opts);
}

void BynavRtcm::GetRtcmMessages(
    std::vector<bynav_gps_msgs::RtcmPtr> &raw_messages) {
  raw_messages.clear();
  raw_messages.insert(raw_messages.end(), rtcm_msgs_.begin(), rtcm_msgs_.end());
  rtcm_msgs_.clear();
}

BynavRtcm::ReadResult BynavRtcm::ProcessData() {
  BynavRtcm::ReadResult read_result = ReadData();

  if (read_result != READ_SUCCESS) {
    return read_result;
  }

  ros::Time stamp = ros::Time::now();
  std::vector<RtcmSentence> rtcm_messages;

  if (!data_buffer_.empty()) {
    rtcm_buffer_.insert(rtcm_buffer_.end(), data_buffer_.begin(),
                        data_buffer_.end());
    data_buffer_.clear();

    std::string remaining_buffer;

    if (!ExtractRtcmMessages(rtcm_buffer_, rtcm_messages, remaining_buffer)) {
      read_result = READ_PARSE_FAILED;
      error_msg_ = "Parse failure extracting sentences.";
    }

    rtcm_buffer_ = remaining_buffer;

    if (!rtcm_buffer_.empty()) {
      ROS_DEBUG("%lu unparsed bytes left over.", rtcm_buffer_.size());
    }
  }

  for (auto &msg : rtcm_messages) {
    bynav_gps_msgs::RtcmPtr ros_msg =
        boost::make_shared<bynav_gps_msgs::Rtcm>();
    ros_msg->header.stamp = stamp;
    ros_msg->data.insert(ros_msg->data.end(), msg.data.begin(), msg.data.end());
    rtcm_msgs_.push_back(std::move(ros_msg));
  }

  return READ_SUCCESS;
}

bool BynavRtcm::ExtractRtcmMessages(const std::string &input,
                                    std::vector<RtcmSentence> &rtcm_sentences,
                                    std::string &remaining) {
  Rtcm rtcm_;
  size_t remaining_count = 0;

  rtcm_.RegisterBufferCallback(
      [this, &remaining_count, &rtcm_sentences](uint8_t *buf, size_t size,
                                                uint16_t id, uint32_t crc) {
        RtcmSentence sent;
        sent.id = id;
        sent.data = std::vector<uint8_t>(buf, buf + size);
        sent.crc = crc;
        rtcm_sentences.push_back(sent);
        remaining_count += size;
      });
  for (int i = 0; i < input.size(); i++) {
    rtcm_.ReadCB(uint8_t(input[i]));
  }
  if (rtcm_.ParsingMessage() && remaining_count < input.size()) {
    remaining = input.substr(remaining_count);
  }
  return true;
}

} // namespace bynav_gps_driver
