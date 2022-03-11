#ifndef BYNAV_RTCM_H_
#define BYNAV_RTCM_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

#include <bynav_gps_driver/bynav_connection.h>
#include <bynav_gps_driver/rtcm_sentence.h>

#include <bynav_gps_msgs/Rtcm.h>

namespace bynav_gps_driver {

class BynavRtcm : public BynavConnection {
public:
  using BynavConnection::ConnectionType;
  using BynavConnection::ReadResult;

  BynavRtcm();
  virtual ~BynavRtcm() = default;

  void GetRtcmMessages(std::vector<bynav_gps_msgs::RtcmPtr> &rtcm_messages);

  bool Connect(const std::string &device, ConnectionType connection);

  ReadResult ProcessData();

private:
  bool ExtractRtcmMessages(const std::string &input,
                           std::vector<RtcmSentence> &rtcm_sentences,
                           std::string &remaining);

  std::string rtcm_buffer_;

  boost::circular_buffer<bynav_gps_msgs::RtcmPtr> rtcm_msgs_;
};

} // namespace bynav_gps_driver
#endif // BYNAV_RTCM_H_
