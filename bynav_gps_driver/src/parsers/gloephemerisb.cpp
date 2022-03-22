#include <bynav_gps_driver/parsers/gloephemerisb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {
static int decode_gloephemerisb(raw_t *raw) {
  unsigned char *p = raw->buff + TERSUSHLEN;
  geph_t geph = {0};
  char *msg;
  double tow, tof, toff;
  int prn, sat, week;

  trace(3, "decode_gloephemerisb: len=%d\n", raw->len);

  if (raw->len < TERSUSHLEN + 144) {
    trace(2, "tersus gloephemerisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U2(p) - 37;

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d", prn);
  }
  if (!(sat = satno(SYS_GLO, prn))) {
    trace(2, "tersus gloephemerisb prn error: prn=%d\n", prn);
    return -1;
  }
  geph.frq = U2(p + 2) + OFF_FRQNO;
  week = U2(p + 6);
  tow = floor(U4(p + 8) / 1000.0 + 0.5); /* rounded to integer sec */
  toff = U4(p + 12);
  geph.iode = U4(p + 20) & 0x7F;
  geph.svh = U4(p + 24);
  geph.pos[0] = R8(p + 28);
  geph.pos[1] = R8(p + 36);
  geph.pos[2] = R8(p + 44);
  geph.vel[0] = R8(p + 52);
  geph.vel[1] = R8(p + 60);
  geph.vel[2] = R8(p + 68);
  geph.acc[0] = R8(p + 76);
  geph.acc[1] = R8(p + 84);
  geph.acc[2] = R8(p + 92);
  geph.taun = R8(p + 100);
  geph.gamn = R8(p + 116);
  tof = U4(p + 124) - toff; /* glonasst->gpst */
  geph.age = U4(p + 136);
  geph.toe = gpst2time(week, tow);
  tof += floor(tow / 86400.0) * 86400;
  if (tof < tow - 43200.0)
    tof += 86400.0;
  else if (tof > tow + 43200.0)
    tof -= 86400.0;
  geph.tof = gpst2time(week, tof);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (fabs(timediff(geph.toe, raw->nav.geph[prn - 1].toe)) < 1.0 &&
        geph.svh == raw->nav.geph[prn - 1].svh)
      return 0; /* unchanged */
  }
  geph.sat = sat;
  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = sat;
  return 2;
}
} // namespace bynav_gps_driver