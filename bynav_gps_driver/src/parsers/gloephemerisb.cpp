#include <bynav_gps_driver/parsers/gloephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>
#include <bynav_gps_driver/parsers/raw_ros.h>

#include <bynav_gps_driver/parsers/header.h>

#include <cmath>
#include <cstring>

#define OFF_FRQNO -7

namespace bynav_gps_driver {

uint32_t GloephemerisbParser::GetMessageId() const { return MESSAGE_ID; }

const std::string GloephemerisbParser::MESSAGE_NAME = "GLOEPHEMERISB";

const std::string GloephemerisbParser::GetMessageName() const {
  return MESSAGE_NAME;
}

static int decode_gloephemerisb(const unsigned char *raw, size_t len,
                                GloEphemPtr geph) {
  unsigned char *p = const_cast<unsigned char *>(raw);
  double tow, tof, toff;
  int prn, sat, week;

  if (len < 144) {
    return -1;
  }
  prn = U2(p) - 37;

  if (!(sat = sat_no(SYS_GLO, prn))) {
    return -1;
  }

  geph->freqo = U2(p + 2) + OFF_FRQNO;
  week = U2(p + 6);
  tow = floor(U4(p + 8) / 1000.0 + 0.5);
  toff = U4(p + 12);
  geph->iode = U4(p + 20) & 0x7F;
  geph->health = (U4(p + 24) < 4) ? 0 : 1;
  geph->pos[0] = R8(p + 28);
  geph->pos[1] = R8(p + 36);
  geph->pos[2] = R8(p + 44);
  geph->vel[0] = R8(p + 52);
  geph->vel[1] = R8(p + 60);
  geph->vel[2] = R8(p + 68);
  geph->acc[0] = R8(p + 76);
  geph->acc[1] = R8(p + 84);
  geph->acc[2] = R8(p + 92);
  geph->tau_n = R8(p + 100);
  geph->delta_tau_n = R8(p + 108);
  geph->gamma = R8(p + 116);
  tof = U4(p + 124) - toff;
  geph->age = U4(p + 136);
  geph->toe = gpst2time(week, tow);
  tof += floor(tow / 86400.0) * 86400;
  if (tof < tow - 43200.0)
    tof += 86400.0;
  else if (tof > tow + 43200.0)
    tof -= 86400.0;
  geph->tof = gpst2time(week, tof);

  geph->sat = sat;
  return 2;
}

bynav_gps_msgs::GnssGloEphemMsgPtr
GloephemerisbParser::ParseBinary(const BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected GLOEPHEMERISB message length: "
          << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  bynav_gps_msgs::GnssGloEphemMsgPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::GnssGloEphemMsg>();

  bynav_gps_msgs::BynavMessageHeader bynav_msg_header;
  
  HeaderParser header_parser;
  bynav_msg_header = header_parser.ParseBinary(bin_msg);
  bynav_msg_header.message_name = MESSAGE_NAME;

  GloEphemPtr geph = std::make_shared<GloEphem>();

  int ret =
      decode_gloephemerisb(bin_msg.data_.data(), bin_msg.data_.size(), geph);
  if (ret < 0) {
    std::stringstream error;
    error << "Unexpected BDSEPHEMERISB message length: "
          << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  *ros_msg = glo_ephem2msg(geph);
  ros_msg->bynav_msg_header = bynav_msg_header;
  return ros_msg;
}

} // namespace bynav_gps_driver