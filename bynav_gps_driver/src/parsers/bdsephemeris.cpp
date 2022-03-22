#include <bynav_gps_driver/parsers/bdsephemeris.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {

static int decode_bdsephemerisb(raw_t *raw) {
  eph_t eph = {0};
  unsigned char *p = raw->buff + TERSUSHLEN;
  double ura, sqrtA;
  char *msg;
  int prn, toc;

  trace(3, "decode_bdsephemerisb: len=%d\n", raw->len);

  if (raw->len < TERSUSHLEN + 196) {
    trace(2, "tersus bdsephemrisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p) - 160;
  p += 4;
  eph.week = U4(p);
  p += 4;
  ura = R8(p);
  p += 8;
  eph.svh = U4(p) & 1;
  p += 4;
  eph.tgd[0] = R8(p);
  p += 8; /* TGD1 for B1 (s) */
  eph.tgd[1] = R8(p);
  p += 8; /* TGD2 for B2 (s) */
  eph.iodc = U4(p);
  p += 4; /* AODC */
  toc = U4(p);
  p += 4;
  eph.f0 = R8(p);
  p += 8;
  eph.f1 = R8(p);
  p += 8;
  eph.f2 = R8(p);
  p += 8;
  eph.iode = U4(p);
  p += 4; /* AODE */
  eph.toes = U4(p);
  p += 4;
  sqrtA = R8(p);
  p += 8;
  eph.e = R8(p);
  p += 8;
  eph.omg = R8(p);
  p += 8;
  eph.deln = R8(p);
  p += 8;
  eph.M0 = R8(p);
  p += 8;
  eph.OMG0 = R8(p);
  p += 8;
  eph.OMGd = R8(p);
  p += 8;
  eph.i0 = R8(p);
  p += 8;
  eph.idot = R8(p);
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
  eph.A = sqrtA * sqrtA;
  eph.sva = uraindex(ura);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d iod=%3d toes=%6.0f", prn, eph.iode, eph.toes);
  }
  if (!(eph.sat = satno(SYS_CMP, prn))) {
    trace(2, "tersus bdsephemeris satellite error: prn=%d\n", prn);
    return -1;
  }
  eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
  eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
  eph.ttr = raw->time;

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