#include <bynav_gps_driver/parsers/qzssrawephemb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {

static int decode_qzssrawephemb(raw_t *raw) {
  unsigned char *p = raw->buff + OEM4HLEN, *q;
  eph_t eph = {0};
  char *msg;
  int i, prn, id, sat;

  trace(3, "decode_qzssrawephemb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 44) {
    trace(2, "oem4 qzssrawephemb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  id = U4(p + 4);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d id=%d", prn, id);
  }
  if (!(sat = satno(SYS_QZS, prn))) {
    trace(2, "oem4 qzssrawephemb satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (id < 1 || 3 < id)
    return 0;

  q = raw->subfrm[sat - 1] + (id - 1) * 30;
  for (i = 0; i < 30; i++)
    *q++ = p[8 + i];

  if (id < 3)
    return 0;
  if (decode_frame(raw->subfrm[sat - 1], &eph, NULL, NULL, NULL, NULL) != 1 ||
      decode_frame(raw->subfrm[sat - 1] + 30, &eph, NULL, NULL, NULL, NULL) !=
          2 ||
      decode_frame(raw->subfrm[sat - 1] + 60, &eph, NULL, NULL, NULL, NULL) !=
          3) {
    return 0;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iodc == raw->nav.eph[sat - 1].iodc &&
        eph.iode == raw->nav.eph[sat - 1].iode)
      return 0; /* unchanged */
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  trace(4, "decode_qzssrawephemb: sat=%2d\n", sat);
  return 2;
}
} // namespace bynav_gps_driver