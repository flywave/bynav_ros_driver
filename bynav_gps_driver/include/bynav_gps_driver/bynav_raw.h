#ifndef BYNAV_RAW_H_
#define BYNAV_RAW_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

#include <bynav_gps_driver/bynav_connection.h>

#include <bynav_gps_msgs/Nav.h>

namespace bynav_gps_driver {

class BynavRaw : public BynavConnection {
public:
  using BynavConnection::ConnectionType;
  using BynavConnection::ReadResult;

  BynavRaw();
  virtual ~BynavRaw() = default;

  bool Connect(const std::string &device, ConnectionType connection);

  ReadResult ProcessData();

private:
  std::string nav_buffer_;
};

} // namespace bynav_gps_driver
#endif // BYNAV_RAW_H_
