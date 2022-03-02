#include <bynav_gps_driver/bynav_connection.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sstream>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

namespace bynav_gps_driver {

BynavConnection::BynavConnection()
    : connection_(SERIAL), is_connected_(false), serial_baud_(115200),
      tcp_socket_(io_service_) {}

BynavConnection::~BynavConnection() { Disconnect(); }

bool BynavConnection::Connect(const std::string &device,
                              ConnectionType connection,
                              BynavMessageOpts const &opts) {
  Disconnect();

  connection_ = connection;

  if (connection_ == SERIAL) {
    return CreateSerialConnection(device, opts);
  } else if (connection_ == TCP || connection_ == UDP) {
    return CreateIpConnection(device, opts);
  }

  error_msg_ = "Invalid connection type.";

  return false;
}

BynavConnection::ConnectionType
BynavConnection::ParseConnection(const std::string &connection) {
  if (connection == "serial") {
    return SERIAL;
  } else if (connection == "udp") {
    return UDP;
  } else if (connection == "tcp") {
    return TCP;
  }

  return INVALID;
}

void BynavConnection::Disconnect() {
  if (connection_ == SERIAL) {
    serial_.Close();
  } else if (connection_ == TCP) {
    tcp_socket_.close();
  } else if (connection_ == UDP) {
    if (udp_socket_) {
      udp_socket_->close();
      udp_socket_.reset();
    }
    if (udp_endpoint_) {
      udp_endpoint_.reset();
    }
  }
  is_connected_ = false;
}

void BynavConnection::SetSerialBaud(int32_t serial_baud) {
  ROS_INFO("Serial baud rate : %d", serial_baud);
  serial_baud_ = serial_baud;
}

bool BynavConnection::Write(const std::string &command) {
  std::vector<uint8_t> bytes(command.begin(), command.end());

  if (connection_ == SERIAL) {
    int32_t written = serial_.Write(bytes);
    if (written != (int32_t)command.length()) {
      ROS_ERROR("Failed to send command: %s", command.c_str());
    }
    return written == (int32_t)command.length();
  } else if (connection_ == TCP || connection_ == UDP) {
    boost::system::error_code error;
    try {
      size_t written;
      if (connection_ == TCP) {
        written =
            boost::asio::write(tcp_socket_, boost::asio::buffer(bytes), error);
      } else {
        written = udp_socket_->send_to(boost::asio::buffer(bytes),
                                       *udp_endpoint_, 0, error);
      }
      if (error) {
        ROS_ERROR("Error writing TCP data: %s", error.message().c_str());
        Disconnect();
      }
      ROS_DEBUG("Wrote %lu bytes.", written);
      return written == (int32_t)command.length();
    } catch (std::exception &e) {
      Disconnect();
      ROS_ERROR("Exception writing TCP data: %s", e.what());
    }
  }

  return false;
}

bool BynavConnection::CreateSerialConnection(const std::string &device,
                                             BynavMessageOpts const &opts) {
  swri_serial_util::SerialConfig config;
  config.baud = serial_baud_;
  config.parity = swri_serial_util::SerialConfig::NO_PARITY;
  config.flow_control = false;
  config.data_bits = 8;
  config.stop_bits = 1;
  config.low_latency_mode = false;
  config.writable = true;

  bool success = serial_.Open(device, config);

  if (success) {
    is_connected_ = true;
    if (!Configure(opts)) {
      ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
                "device may not be functioning as expected; however, the "
                "driver may still function correctly if the port has already "
                "been pre-configured.");
    }
  } else {
    error_msg_ = serial_.ErrorMsg();
  }

  return success;
}

bool BynavConnection::CreateIpConnection(const std::string &endpoint,
                                         BynavMessageOpts const &opts) {
  std::string ip;
  std::string port;
  uint16_t num_port;
  size_t sep_pos = endpoint.find(':');
  if (sep_pos == std::string::npos || sep_pos == endpoint.size() - 1) {
    ROS_INFO("Using default port.");
    std::stringstream ss;
    if (connection_ == TCP) {
      num_port = DEFAULT_TCP_PORT;
    } else {
      num_port = DEFAULT_UDP_PORT;
    }
    ss << num_port;
    port = ss.str();
  } else {
    port = endpoint.substr(sep_pos + 1);
  }

  if (sep_pos != 0) {
    ip = endpoint.substr(0, sep_pos);
  }

  try {
    if (!ip.empty()) {
      if (connection_ == TCP) {
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(ip, port);
        boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

        boost::asio::connect(tcp_socket_, iter);
        ROS_INFO("Connecting via TCP to %s:%s", ip.c_str(), port.c_str());
      } else {
        boost::asio::ip::udp::resolver resolver(io_service_);
        boost::asio::ip::udp::resolver::query query(ip, port);
        udp_endpoint_ = boost::make_shared<boost::asio::ip::udp::endpoint>(
            *resolver.resolve(query));
        udp_socket_.reset(new boost::asio::ip::udp::socket(io_service_));
        udp_socket_->open(boost::asio::ip::udp::v4());
        ROS_INFO("Connecting via UDP to %s:%s", ip.c_str(), port.c_str());
      }
    } else {
      auto port_num = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));
      if (connection_ == TCP) {
        boost::asio::ip::tcp::acceptor acceptor(
            io_service_, boost::asio::ip::tcp::endpoint(
                             boost::asio::ip::tcp::v4(), port_num));
        ROS_INFO("Listening on TCP port %s", port.c_str());
        acceptor.accept(tcp_socket_);
        ROS_INFO("Accepted TCP connection from client: %s",
                 tcp_socket_.remote_endpoint().address().to_string().c_str());
      } else {
        udp_socket_.reset(new boost::asio::ip::udp::socket(
            io_service_, boost::asio::ip::udp::endpoint(
                             boost::asio::ip::udp::v4(), port_num)));
        boost::array<char, 1> recv_buf;
        udp_endpoint_ = boost::make_shared<boost::asio::ip::udp::endpoint>();
        boost::system::error_code error;

        ROS_INFO("Listening on UDP port %s", port.c_str());
        udp_socket_->receive_from(boost::asio::buffer(recv_buf), *udp_endpoint_,
                                  0, error);
        if (error && error != boost::asio::error::message_size) {
          throw boost::system::system_error(error);
        }

        ROS_INFO("Accepted UDP connection from client: %s",
                 udp_endpoint_->address().to_string().c_str());
      }
    }
  } catch (std::exception &e) {
    error_msg_ = e.what();
    ROS_ERROR("Unable to connect: %s", e.what());
    return false;
  }

  is_connected_ = true;

  if (Configure(opts)) {
    ROS_INFO("Configured GPS.");
  } else {
    ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
              "device may not be functioning as expected; however, the "
              "driver may still function correctly if the port has already "
              "been pre-configured.");
  }

  return true;
}

bool BynavConnection::Configure(BynavMessageOpts const &opts) {
  bool configured = true;
  std::stringstream unlogport;
  unlogport << "unlogall\r\n";
  configured = configured && Write(unlogport.str());

  for (const auto &option : opts) {
    std::stringstream command;
    command << std::setprecision(3);
    if (option.second < 0.0) {
      command << "log " << option.first << " onchanged\r\n";
    } else {
      command << "log " << option.first << " ontime " << option.second
              << "\r\n";
    }
    configured = configured && Write(command.str());
  }

  return configured;
}

BynavConnection::ReadResult BynavConnection::ReadData() {
  if (connection_ == SERIAL) {
    swri_serial_util::SerialPort::Result result =
        serial_.ReadBytes(data_buffer_, 0, 1000);

    if (result == swri_serial_util::SerialPort::ERROR) {
      error_msg_ = serial_.ErrorMsg();
      return READ_ERROR;
    } else if (result == swri_serial_util::SerialPort::TIMEOUT) {
      error_msg_ = "Timed out waiting for serial device.";
      return READ_TIMEOUT;
    } else if (result == swri_serial_util::SerialPort::INTERRUPTED) {
      error_msg_ = "Interrupted during read from serial device.";
      return READ_INTERRUPTED;
    }

    return READ_SUCCESS;
  } else if (connection_ == TCP || connection_ == UDP) {
    try {
      boost::system::error_code error;
      size_t len;

      if (connection_ == TCP) {
        len = tcp_socket_.read_some(boost::asio::buffer(socket_buffer_), error);
      } else {
        boost::asio::ip::udp::endpoint remote_endpoint;
        len = udp_socket_->receive_from(boost::asio::buffer(socket_buffer_),
                                        remote_endpoint);
      }
      data_buffer_.insert(data_buffer_.end(), socket_buffer_.begin(),
                          socket_buffer_.begin() + len);
      if (error) {
        error_msg_ = error.message();
        Disconnect();
        return READ_ERROR;
      }
      return READ_SUCCESS;
    } catch (std::exception &e) {
      ROS_WARN("TCP connection error: %s", e.what());
    }
  }

  error_msg_ = "Unsupported connection type.";

  return READ_ERROR;
}

} // namespace bynav_gps_driver
