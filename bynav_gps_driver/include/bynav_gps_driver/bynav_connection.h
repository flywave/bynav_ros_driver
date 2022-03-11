#ifndef BYNAV_CONNECTION_H_
#define BYNAV_CONNECTION_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

namespace bynav_gps_driver {

typedef std::map<std::string, double> BynavMessageOpts;

class BynavConnection {
public:
  enum ConnectionType { SERIAL, TCP, UDP, INVALID };

  enum ReadResult {
    READ_SUCCESS = 0,
    READ_INSUFFICIENT_DATA = 1,
    READ_TIMEOUT = 2,
    READ_INTERRUPTED = 3,
    READ_ERROR = -1,
    READ_PARSE_FAILED = -2
  };

  BynavConnection();
  virtual ~BynavConnection();

  bool Connect(const std::string &device, ConnectionType connection,
               BynavMessageOpts const &opts);

  void Disconnect();

  std::string ErrorMsg() const { return error_msg_; }

  bool IsConnected() { return is_connected_; }

  static ConnectionType ParseConnection(const std::string &connection);

  void SetSerialBaud(int32_t serial_baud);

  bool Write(const std::string &command);

  static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
  static constexpr uint16_t DEFAULT_UDP_PORT = 3002;

  static constexpr size_t MAX_BUFFER_SIZE = 100;
  static constexpr size_t SYNC_BUFFER_SIZE = 10;

  virtual bool Configure(BynavMessageOpts const &opts);

  virtual ReadResult ReadData();

protected:
  bool CreateIpConnection(const std::string &endpoint,
                          BynavMessageOpts const &opts);

  bool CreateSerialConnection(const std::string &device,
                              BynavMessageOpts const &opts);

  ConnectionType connection_;

  std::string error_msg_;

  bool is_connected_;

  int32_t serial_baud_;
  swri_serial_util::SerialPort serial_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket tcp_socket_;
  boost::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;
  boost::shared_ptr<boost::asio::ip::udp::endpoint> udp_endpoint_;

  std::vector<uint8_t> data_buffer_;
  boost::array<uint8_t, 10000> socket_buffer_;
};

} // namespace bynav_gps_driver
#endif // BYNAV_CONNECTION_H_
