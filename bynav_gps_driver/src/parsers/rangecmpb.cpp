#include <bynav_gps_driver/parsers/rangecmpb.h>
#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>
#include <cstring>

namespace bynav_gps_driver {

static int decode_trackstat(unsigned int stat, int *sys, int *code, int *track,
                            int *plock, int *clock, int *parity, int *halfc) {
  int satsys, sigtype, freq = 0;

  *track = stat & 0x1F;
  *plock = (stat >> 10) & 1;
  *parity = (stat >> 11) & 1;
  *clock = (stat >> 12) & 1;
  satsys = (stat >> 16) & 7;
  *halfc = (stat >> 28) & 1;
  sigtype = (stat >> 21) & 0x1F;

  switch (satsys) {
  case 0:
    *sys = SYS_GPS;
    break;
  case 1:
    *sys = SYS_GLO;
    break;
  case 2:
    *sys = SYS_SBS;
    break;
  case 3:
    *sys = SYS_GAL;
    break;
  case 4:
    *sys = SYS_CMP;
    break;
  case 5:
    *sys = SYS_QZS;
    break;
  default:
    // trace(2, "tersus unknown system: sys=%d\n", satsys);
    return -1;
  }
  if (*sys == SYS_GPS || *sys == SYS_QZS) {
    switch (sigtype) {
    case 0:
      freq = 0;
      *code = CODE_L1C;
      break; /* L1C/A */
    case 5:
      freq = 0;
      *code = CODE_L1P;
      break; /* L1P */
    case 9:
      freq = 1;
      *code = CODE_L2D;
      break; /* L2Pcodeless */
    case 14:
      freq = 2;
      *code = CODE_L5Q;
      break; /* L5Q */
    case 17:
      freq = 1;
      *code = CODE_L2X;
      break; /* L2C(M+L) */
    default:
      freq = -1;
      break;
    }
  } else if (*sys == SYS_GLO) {
    switch (sigtype) {
    case 0:
      freq = 0;
      *code = CODE_L1C;
      break; /* L1C/A */
    case 1:
      freq = 1;
      *code = CODE_L2C;
      break; /* L2C/A (OEM6) */
    case 5:
      freq = 1;
      *code = CODE_L2P;
      break; /* L2P */
    default:
      freq = -1;
      break;
    }
  } else if (*sys == SYS_GAL) {
    switch (sigtype) {
    case 1:
      freq = 0;
      *code = CODE_L1B;
      break; /* E1B */
    case 2:
      freq = 0;
      *code = CODE_L1C;
      break; /* E1C */
    case 12:
      freq = 2;
      *code = CODE_L5Q;
      break; /* E5aQ */
    case 17:
      freq = 1;
      *code = CODE_L7Q;
      break; /* E5bQ */
    case 20:
      freq = 5;
      *code = CODE_L8Q;
      break; /* AltBOCQ */
    default:
      freq = -1;
      break;
    }
  } else if (*sys == SYS_CMP) {
    switch (sigtype) {
    case 0:
      freq = 0;
      *code = CODE_L1I;
      break; /* B1 with D1 */
    case 1:
      freq = 1;
      *code = CODE_L7I;
      break; /* B2 with D1 */
    case 4:
      freq = 0;
      *code = CODE_L1I;
      break; /* B1 with D2 */
    case 5:
      freq = 1;
      *code = CODE_L7I;
      break; /* B2 with D2 */
    case 21:
      freq = 2;
      *code = CODE_L6I;
      break; /* B3 */
    default:
      freq = -1;
      break;
    }
  } else if (*sys == SYS_SBS) {
    switch (sigtype) {
    case 0:
      freq = 0;
      *code = CODE_L1C;
      break; /* L1C/A */
    case 6:
      freq = 2;
      *code = CODE_L5I;
      break; /* L5I */
    default:
      freq = -1;
      break;
    }
  }
  if (freq < 0) {
    // trace(2, "tersus signal type error: sys=%d sigtype=%d\n", *sys, sigtype);
    return -1;
  }
  return freq;
}

static int checkpri(const char *opt, int sys, int code, int freq) {
  int nex = NEXOBS; /* number of extended obs data */

  if (sys == SYS_GPS) {
    if (strstr(opt, "-GL1P") && freq == 0)
      return code == CODE_L1P ? 0 : -1;
    if (strstr(opt, "-GL2X") && freq == 1)
      return code == CODE_L2X ? 1 : -1;
    if (code == CODE_L1P)
      return nex < 1 ? -1 : NFREQ;
    if (code == CODE_L2X)
      return nex < 2 ? -1 : NFREQ + 1;
  } else if (sys == SYS_GLO) {
    if (strstr(opt, "-RL2C") && freq == 1)
      return code == CODE_L2C ? 1 : -1;
    if (code == CODE_L2C)
      return nex < 1 ? -1 : NFREQ;
  } else if (sys == SYS_GAL) {
    if (strstr(opt, "-EL1B") && freq == 0)
      return code == CODE_L1B ? 0 : -1;
    if (code == CODE_L1B)
      return nex < 1 ? -1 : NFREQ;
    if (code == CODE_L8Q)
      return nex < 3 ? -1 : NFREQ + 2;
  }
  return freq < NFREQ ? freq : -1;
}

static int decode_rangecmpb(raw_t *raw) {
  double psr, adr, adr_rolls, lockt, tt, dop, snr, wavelen;
  int i, index, nobs, prn, sat, sys, code, freq, pos;
  int track, plock, clock, parity, halfc, lli;
  char *msg;
  unsigned char *p = raw->buff + TERSUSHLEN;

  trace(3, "decode_rangecmpb: len=%d\n", raw->len);

  nobs = U4(p);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " nobs=%2d", nobs);
  }
  if (raw->len < TERSUSHLEN + 4 + nobs * 24) {
    trace(2, "tersus rangecmpb length error: len=%d nobs=%d\n", raw->len, nobs);
    return -1;
  }
  for (i = 0, p += 4; i < nobs; i++, p += 24) {

    /* decode tracking status */
    if ((freq = decode_trackstat(U4(p), &sys, &code, &track, &plock, &clock,
                                 &parity, &halfc)) < 0)
      continue;

    /* obs position */
    if ((pos = checkpri(raw->opt, sys, code, freq)) < 0)
      continue;

    prn = U1(p + 17);
    if (sys == SYS_GLO)
      prn -= 37;
    else if (sys == SYS_CMP)
      prn -= 160;

    if (!(sat = satno(sys, prn))) {
      trace(3, "tersus rangecmpb satellite number error: sys=%d,prn=%d\n", sys,
            prn);
      continue;
    }
    if (sys == SYS_GLO && !parity)
      continue; /* invalid if GLO parity unknown */

    dop = exsign(U4(p + 4) & 0xFFFFFFF, 28) / 256.0;
    psr = (U4(p + 7) >> 4) / 128.0 + U1(p + 11) * 2097152.0;

    if ((wavelen = satwavelen(sat, freq, &raw->nav)) <= 0.0) {
      if (sys == SYS_GLO)
        wavelen = CLIGHT / (freq == 0 ? FREQ1_GLO : FREQ2_GLO);
      else
        wavelen = lam_carr[freq];
    }
    adr = I4(p + 12) / 256.0;
    adr_rolls = (psr / wavelen + adr) / MAXVAL;
    adr = -adr + MAXVAL * floor(adr_rolls + (adr_rolls <= 0 ? -0.5 : 0.5));

    lockt = (U4(p + 18) & 0x1FFFFF) / 32.0; /* lock time */

    if (raw->tobs[sat - 1][pos].time != 0) {
      tt = timediff(raw->time, raw->tobs[sat - 1][pos]);
      lli = (lockt < 65535.968 && lockt - raw->lockt[sat - 1][pos] + 0.05 <= tt)
                ? LLI_SLIP
                : 0;
    } else {
      lli = 0;
    }
    if (!parity)
      lli |= LLI_HALFC;
    if (halfc)
      lli |= LLI_HALFA;
    raw->tobs[sat - 1][pos] = raw->time;
    raw->lockt[sat - 1][pos] = lockt;
    raw->halfc[sat - 1][pos] = halfc;

    snr = ((U2(p + 20) & 0x3FF) >> 5) + 20.0;
    if (!clock)
      psr = 0.0; /* code unlock */
    if (!plock)
      adr = dop = 0.0; /* phase unlock */

    if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
      raw->obs.n = 0;
    }
    if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
      raw->obs.data[index].L[pos] = adr;
      raw->obs.data[index].P[pos] = psr;
      raw->obs.data[index].D[pos] = (float)dop;
      raw->obs.data[index].SNR[pos] =
          0.0 <= snr && snr < 255.0 ? (unsigned char)(snr * 4.0 + 0.5) : 0;
      raw->obs.data[index].LLI[pos] = (unsigned char)lli;
      raw->obs.data[index].code[pos] = code;
    }
  }
  return 1;
}
} // namespace bynav_gps_driver