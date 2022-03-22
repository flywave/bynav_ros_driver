#include <bynav_gps_driver/parsers/gpsphemb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {
static int decode_gpsephemb(raw_t *raw) {
  unsigned char *p = raw->buff + TERSUSHLEN;
  eph_t eph = {0};
  char *msg;
  double tow, toc, n, ura, tt;
  int prn, week, zweek, iode2, as;

  trace(3, "decode_gpsephemb: len=%d\n", raw->len);

  if (raw->len < TERSUSHLEN + 224) {
    trace(2, "tersus gpsephemb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U2(p);
  p += 4;

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d", prn);
  }
  if (!(eph.sat = satno(SYS_GPS, prn))) {
    trace(2, "tersus gpsephemb prn error: prn=%d\n", prn);
    return -1;
  }
  tow = R8(p);
  p += 8;
  eph.svh = (int)U4(p);
  p += 4;
  eph.iode = (int)U4(p);
  p += 4;
  iode2 = (int)U4(p);
  p += 4;
  week = (int)U4(p);
  p += 4;
  zweek = U4(p);
  p += 4;
  eph.toes = R8(p);
  p += 8;
  eph.A = R8(p);
  p += 8;
  eph.deln = R8(p);
  p += 8;
  eph.M0 = R8(p);
  p += 8;
  eph.e = R8(p);
  p += 8;
  eph.omg = R8(p);
  p += 8;
  eph.cuc = R8(p);
  p += 8;
  eph.cus = R8(p);
  p += 8;
  eph.crc = R8(p);
  p += 8;
  eph.crs = R8(p);
  p += 8;
  eph.cic = R8(p);
  p += 8;
  eph.cis = R8(p);
  p += 8;
  eph.i0 = R8(p);
  p += 8;
  eph.idot = R8(p);
  p += 8;
  eph.OMG0 = R8(p);
  p += 8;
  eph.OMGd = R8(p);
  p += 8;
  eph.iodc = (int)U4(p);
  p += 4;
  toc = R8(p);
  p += 8;
  eph.tgd[0] = R8(p);
  p += 8;
  eph.f0 = R8(p);
  p += 8;
  eph.f1 = R8(p);
  p += 8;
  eph.f2 = R8(p);
  p += 8;
  as = (int)U4(p);
  p += 4; /* AS-ON */
  n = R8(p);
  p += 8;
  ura = R8(p);
  p += 8;

  if (eph.iode != iode2) {
    trace(2, "tersus gpsephemb iode error: iode=%d %d\n", eph.iode, iode2);
    return -1;
  }
  eph.week = adjgpsweek(week);
  eph.toe = gpst2time(eph.week, eph.toes);
  tt = timediff(eph.toe, raw->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt > 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = adjweek(eph.toe, tow);
  eph.sva = uraindex(ura);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (timediff(raw->nav.eph[eph.sat - 1].toe, eph.toe) == 0.0 &&
        raw->nav.eph[eph.sat - 1].iode == eph.iode &&
        raw->nav.eph[eph.sat - 1].iodc == eph.iodc)
      return 0; /* unchanged */
  }
  raw->nav.eph[eph.sat - 1] = eph;
  raw->ephsat = eph.sat;
  return 2;
}
} // namespace bynav_gps_driver