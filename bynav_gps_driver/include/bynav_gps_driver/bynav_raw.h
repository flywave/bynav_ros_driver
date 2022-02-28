#ifndef BYNAV_BYNAV_RAW_H_
#define BYNAV_BYNAV_RAW_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
 

#include <swri_serial_util/serial_port.h>

#include <bynav_gps_driver/bynav_connection.h>

#include <bynav_gps_msgs/Raw.h>

namespace bynav_gps_driver {

class BynavRaw : public BynavConnection {
public:
  using BynavConnection::ConnectionType;
  using BynavConnection::ReadResult;

  BynavRaw();
  virtual ~BynavRaw() = default;

  void GetRawMessages(std::vector<bynav_gps_msgs::RawPtr> &raw_messages);

  bool Connect(const std::string &device, ConnectionType connection);

  ReadResult ProcessData();

private:
  std::string raw_buffer_;

  boost::circular_buffer<bynav_gps_msgs::RawPtr> raw_msgs_;
};

} // namespace bynav_gps_driver
#endif // BYNAV_BYNAV_RAW_H_
