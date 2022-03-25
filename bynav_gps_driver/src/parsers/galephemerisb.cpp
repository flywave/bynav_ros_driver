#include <bynav_gps_driver/parsers/galephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>
#include <bynav_gps_driver/parsers/raw_ros.h>

#include <bynav_gps_driver/parsers/header.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {

uint32_t GaleephemerisbParser::GetMessageId() const { return MESSAGE_ID; }

const std::string GaleephemerisbParser::MESSAGE_NAME = "GALEEPHEMERISB";

const std::string GaleephemerisbParser::GetMessageName() const {
  return MESSAGE_NAME;
}

static int decode_galephemerisb(const unsigned char *raw, size_t len,
                                EphemPtr eph, gtime_t time, int sel_nav = 0) {
  unsigned char *p = const_cast<unsigned char *>(raw);
  double tow, sqrtA, af0_fnav, af1_fnav, af2_fnav, af0_inav, af1_inav, af2_inav,
      tt;
  char *msg;
  int prn, rcv_fnav, rcv_inav, svh_e1b, svh_e5a, svh_e5b, dvs_e1b, dvs_e5a,
      dvs_e5b;
  uint32_t toc_fnav, toc_inav, week = 0;

  if (len < 220) {
    return -1;
  }
  prn = U4(p);
  p += 4;
  rcv_fnav = U4(p) & 1;
  p += 4;
  rcv_inav = U4(p) & 1;
  p += 4;
  svh_e1b = U1(p) & 3;
  p += 1;
  svh_e5a = U1(p) & 3;
  p += 1;
  svh_e5b = U1(p) & 3;
  p += 1;
  dvs_e1b = U1(p) & 1;
  p += 1;
  dvs_e5a = U1(p) & 1;
  p += 1;
  dvs_e5b = U1(p) & 1;
  p += 1;
  eph->sindex = U1(p);
  p += 1 + 1; /* SISA index */
  eph->iode = U4(p);
  p += 4; /* IODNav */
  eph->toes = U4(p);
  p += 4;
  sqrtA = R8(p);
  p += 8;
  eph->delta_n = R8(p);
  p += 8;
  eph->M0 = R8(p);
  p += 8;
  eph->e = R8(p);
  p += 8;
  eph->omg = R8(p);
  p += 8;
  eph->cuc = R8(p);
  p += 8;
  eph->cus = R8(p);
  p += 8;
  eph->crc = R8(p);
  p += 8;
  eph->crs = R8(p);
  p += 8;
  eph->cic = R8(p);
  p += 8;
  eph->cis = R8(p);
  p += 8;
  eph->i0 = R8(p);
  p += 8;
  eph->i_dot = R8(p);
  p += 8;
  eph->OMG0 = R8(p);
  p += 8;
  eph->OMG_dot = R8(p);
  p += 8;
  toc_fnav = U4(p);
  p += 4;
  af0_fnav = R8(p);
  p += 8;
  af1_fnav = R8(p);
  p += 8;
  af2_fnav = R8(p);
  p += 8;
  toc_inav = U4(p);
  p += 4;
  af0_inav = R8(p);
  p += 8;
  af1_inav = R8(p);
  p += 8;
  af2_inav = R8(p);
  p += 8;
  eph->tgd[0] = R8(p);
  p += 8;
  eph->tgd[1] = R8(p);
  eph->iodc = eph->iode;
  eph->health = (svh_e5b << 7) | (dvs_e5b << 6) | (svh_e5a << 4) |
                (dvs_e5a << 3) | (svh_e1b << 1) | dvs_e1b;

  eph->A = sqrtA * sqrtA;
  eph->af0 = sel_nav ? af0_fnav : af0_inav;
  eph->af1 = sel_nav ? af1_fnav : af1_inav;
  eph->af2 = sel_nav ? af2_fnav : af2_inav;

  eph->code = (sel_nav == 0) ? ((1 << 0) | (1 << 9)) : ((1 << 1) | (1 << 8));

  tow = time2gpst(time, &week);
  eph->week = week;
  eph->toe = gpst2time(eph->week, eph->toes);

  tt = time_diff(eph->toe, time);
  if (tt < -302400.0)
    eph->week++;
  else if (tt > 302400.0)
    eph->week--;
  eph->toe = gpst2time(eph->week, eph->toes);
  eph->toc = adjweek(eph->toe, sel_nav ? toc_fnav : toc_inav);
  eph->ttr = adjweek(eph->toe, tow);

  return 2;
}

bynav_gps_msgs::GnssEphemMsgPtr
GaleephemerisbParser::ParseBinary(const BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected GALEEPHEMERISB message length: "
          << bin_msg.data_.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GnssEphemMsgPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::GnssEphemMsg>();

  bynav_gps_msgs::BynavMessageHeader bynav_msg_header;
  HeaderParser header_parser;
  bynav_msg_header = header_parser.ParseBinary(bin_msg);
  bynav_msg_header.message_name = MESSAGE_NAME;

  gtime_t time =
      gpst2time(bynav_msg_header.gps_week_num, bynav_msg_header.gps_seconds);

  EphemPtr eph = std::make_shared<Ephem>();

  int ret = decode_galephemerisb(bin_msg.data_.data(), bin_msg.data_.size(),
                                 eph, time);
  if (ret < 0) {
    std::stringstream error;
    error << "Unexpected BDSEPHEMERISB message length: "
          << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  *ros_msg = ephem2msg(eph);
  ros_msg->bynav_msg_header = bynav_msg_header;
  return ros_msg;
}

} // namespace bynav_gps_driver