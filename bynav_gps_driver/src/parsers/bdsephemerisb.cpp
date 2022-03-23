#include <bynav_gps_driver/parsers/bdsephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

#define HLEN 28
#define SQR(x) ((x) * (x))

namespace bynav_gps_driver {

static int decode_bdsephemerisb(unsigned char *raw, size_t len, EphemPtr eph,
                                gtime_t time) {
  unsigned char *p = raw + HLEN;
  double ura, sqrtA;
  int prn, sat, toc;

  if (len < HLEN + 196) {
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
} // namespace bynav_gps_driver