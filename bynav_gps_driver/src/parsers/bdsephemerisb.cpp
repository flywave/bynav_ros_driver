#include <bynav_gps_driver/parsers/bdsephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>
#include <bynav_gps_driver/parsers/raw_ros.h>

#include <bynav_gps_driver/parsers/header.h>

#include <cmath>
#include <cstring>

#define SQR(x) ((x) * (x))

namespace bynav_gps_driver {

uint32_t BdsephemerisbParser::GetMessageId() const { return MESSAGE_ID; }

const std::string BdsephemerisbParser::MESSAGE_NAME = "BDSEPHEMERISB";

const std::string BdsephemerisbParser::GetMessageName() const {
  return MESSAGE_NAME;
}

static int decode_bdsephemerisb(const unsigned char *raw, size_t len,
                                EphemPtr eph, gtime_t time) {
  unsigned char *p = const_cast<unsigned char *>(raw);
  double ura, sqrtA;
  int prn, sat, toc;

  if (len < 196) {
    return -1;
  }
  prn = U4(p);
  p += 4;
  eph->week = U4(p);
  p += 4;
  ura = R8(p);
  p += 8;
  eph->health = U4(p) & 1;
  p += 4;
  eph->tgd[0] = R8(p);
  p += 8;
  eph->tgd[1] = R8(p);
  p += 8;
  eph->iodc = U4(p);
  p += 4;
  toc = U4(p);
  p += 4;
  eph->af0 = R8(p);
  p += 8;
  eph->af1 = R8(p);
  p += 8;
  eph->af2 = R8(p);
  p += 8;
  eph->iode = U4(p);
  p += 4;
  eph->toes = U4(p);
  p += 4;
  sqrtA = R8(p);
  p += 8;
  eph->e = R8(p);
  p += 8;
  eph->omg = R8(p);
  p += 8;
  eph->delta_n = R8(p);
  p += 8;
  eph->M0 = R8(p);
  p += 8;
  eph->OMG0 = R8(p);
  p += 8;
  eph->OMG_dot = R8(p);
  p += 8;
  eph->i0 = R8(p);
  p += 8;
  eph->i_dot = R8(p);
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

  if (!(sat = sat_no(SYS_CMP, prn))) {
    return -1;
  }
  eph->sat = sat;
  eph->A = SQR(sqrtA);
  eph->ura = ura;
  eph->sindex = uraindex(ura);
  eph->toe = bdt2gpst(bdt2time(eph->week, eph->toes)); /* bdt -> gpst */
  eph->toc = bdt2gpst(bdt2time(eph->week, toc));       /* bdt -> gpst */
  eph->ttr = time;

  return 2;
}

bynav_gps_msgs::GnssEphemMsgPtr
BdsephemerisbParser::ParseBinary(const BinaryMessage &bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected BDSEPHEMERISB message length: "
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

  int ret = decode_bdsephemerisb(bin_msg.data_.data(), bin_msg.data_.size(),
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