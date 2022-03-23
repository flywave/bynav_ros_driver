#include <bynav_gps_driver/parsers/qzssephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

#define HLEN 28

namespace bynav_gps_driver {

static int decode_qzssephemerisb(unsigned char *buff, size_t len, EphemPtr eph,
                                 gtime_t time) {
  unsigned char *p = buff + HLEN;
  char *msg;
  double tow, toc, n, ura, tt;
  uint32_t prn, week, zweek, iode2, as;

  if (len < HLEN + 224) {
    return -1;
  }
  prn = U2(p);
  p += 4;

  if (!(eph->sat = sat_no(SYS_GPS, prn))) {
    return -1;
  }
  tow = R8(p);
  p += 8;
  eph->health = (int)U4(p);
  p += 4;
  eph->iode = (int)U4(p);
  p += 4;
  iode2 = (int)U4(p);
  p += 4;
  week = (int)U4(p);
  p += 4;
  zweek = U4(p);
  p += 4;
  eph->toes = R8(p);
  p += 8;
  eph->A = R8(p);
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
  eph->iodc = (int)U4(p);
  p += 4;
  toc = R8(p);
  p += 8;
  eph->tgd[0] = R8(p);
  p += 8;
  eph->af0 = R8(p);
  p += 8;
  eph->af1 = R8(p);
  p += 8;
  eph->af2 = R8(p);
  p += 8;
  as = (int)U4(p);
  p += 4;
  n = R8(p);
  p += 8;
  ura = R8(p);
  p += 8;

  if (eph->iode != iode2) {
    return -1;
  }
  eph->week = adjgpsweek(week, false);
  eph->toe = gpst2time(eph->week, eph->toes);
  tt = time_diff(eph->toe, time);
  if (tt < -302400.0)
    eph->week++;
  else if (tt > 302400.0)
    eph->week--;
  eph->toe = gpst2time(eph->week, eph->toes);
  eph->toc = gpst2time(eph->week, toc);
  eph->ttr = adjweek(eph->toe, tow);
  eph->sindex = uraindex(ura);

  return 1;
}
} // namespace bynav_gps_driver