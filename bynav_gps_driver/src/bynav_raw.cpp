#include <bynav_gps_driver/bynav_raw.h>

namespace bynav_gps_driver {

BynavRaw::BynavRaw() {}

bool BynavRaw::Connect(const std::string &device, ConnectionType connection) {
  BynavMessageOpts opts;
  opts["GALEEPHEMERISB"] = -1;
  opts["GPSEPHEMB"] = -1;
  opts["GLOEPHEMERISB"] = -1;
  opts["BDSEPHEMERISB"] = -1;
  opts["QZSSEPHEMERISB"] = -1;
  opts["RANGECMPB"] = -1;
  return BynavConnection::Connect(device, connection, opts);
}

ReadResult BynavRaw::ProcessData() {}
} // namespace bynav_gps_driver