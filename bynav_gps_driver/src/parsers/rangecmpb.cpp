#include <bynav_gps_driver/parsers/rangecmpb.h>
#include <bynav_gps_driver/parsers/raw.h>
#include <bynav_gps_driver/parsers/raw_ros.h>

#include <bynav_gps_driver/parsers/header.h>

#include <cmath>
#include <cstring>

#define CLIGHT 299792458.0 /* speed of light (m/s) */

#define LLI_SLIP 0x01   /* LLI: cycle-slip */
#define LLI_HALFC 0x02  /* LLI: half-cycle not resovled */
#define LLI_BOCTRK 0x04 /* LLI: boc tracking of mboc signal */
#define LLI_HALFA 0x40  /* LLI: half-cycle added */
#define LLI_HALFS 0x80  /* LLI: half-cycle subtracted */

#define MAXVAL 8388608.0

#define MINPRNQZS_S 183 /* min satellite PRN number of QZSS L1S */
#define MAXPRNQZS_S 191 /* max satellite PRN number of QZSS L1S */

#define SNR_UNIT 0.001 /* SNR unit (dBHz) */

namespace bynav_gps_driver {

uint32_t RangrcmpbParser::GetMessageId() const { return MESSAGE_ID; }

const std::string RangrcmpbParser::MESSAGE_NAME = "RANGECMPB";

const std::string RangrcmpbParser::GetMessageName() const {
  return MESSAGE_NAME;
}

static char *obscodes[] = {
    /* observation code strings */
    "",   "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E", /*  0- 9 */
    "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P", /* 10-19 */
    "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X", /* 20-29 */
    "6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8L", "8Q", "8X", /* 30-39 */
    "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A", /* 40-49 */
    "5B", "5C", "9A", "9B", "9C", "9X", "1D", "5D", "5P", "5Z", /* 50-59 */
    "6E", "7D", "7P", "7Z", "8D", "8P", "4A", "4B", "4X", ""    /* 60-69 */
};
static int sig2code(int sys, int sigtype) {
  if (sys == SYS_GPS) {
    switch (sigtype) {
    case 0:
      return CODE_L1C; /* L1C/A */
    case 5:
      return CODE_L2P; /* L2P    (OEM7) */
    case 9:
      return CODE_L2W; /* L2P(Y),semi-codeless */
    case 14:
      return CODE_L5Q; /* L5Q    (OEM6) */
    case 16:
      return CODE_L1L; /* L1C(P) (OEM7) */
    case 17:
      return CODE_L2S; /* L2C(M) (OEM7) */
    }
  } else if (sys == SYS_GLO) {
    switch (sigtype) {
    case 0:
      return CODE_L1C; /* L1C/A */
    case 1:
      return CODE_L2C; /* L2C/A (OEM6) */
    case 5:
      return CODE_L2P; /* L2P */
    case 6:
      return CODE_L3Q; /* L3Q   (OEM7) */
    }
  } else if (sys == SYS_GAL) {
    switch (sigtype) {
    case 2:
      return CODE_L1C; /* E1C  (OEM6) */
    case 6:
      return CODE_L6B; /* E6B  (OEM7) */
    case 7:
      return CODE_L6C; /* E6C  (OEM7) */
    case 12:
      return CODE_L5Q; /* E5aQ (OEM6) */
    case 17:
      return CODE_L7Q; /* E5bQ (OEM6) */
    case 20:
      return CODE_L8Q; /* AltBOCQ (OEM6) */
    }
  } else if (sys == SYS_QZS) {
    switch (sigtype) {
    case 0:
      return CODE_L1C; /* L1C/A */
    case 14:
      return CODE_L5Q; /* L5Q    (OEM6) */
    case 16:
      return CODE_L1L; /* L1C(P) (OEM7) */
    case 17:
      return CODE_L2S; /* L2C(M) (OEM7) */
    case 27:
      return CODE_L6L; /* L6P    (OEM7) */
    }
  } else if (sys == SYS_CMP) {
    switch (sigtype) {
    case 0:
      return CODE_L2I; /* B1I with D1 (OEM6) */
    case 1:
      return CODE_L7I; /* B2I with D1 (OEM6) */
    case 2:
      return CODE_L6I; /* B3I with D1 (OEM7) */
    case 4:
      return CODE_L2I; /* B1I with D2 (OEM6) */
    case 5:
      return CODE_L7I; /* B2I with D2 (OEM6) */
    case 6:
      return CODE_L6I; /* B3I with D2 (OEM7) */
    case 7:
      return CODE_L1P; /* B1C(P) (OEM7) */
    case 9:
      return CODE_L5P; /* B2a(P) (OEM7) */
    case 11:
      return CODE_L7D; /* B2b(I) (OEM7,F/W 7.08) */
    }
  } else if (sys == SYS_IRN) {
    switch (sigtype) {
    case 0:
      return CODE_L5A; /* L5 (OEM7) */
    }
  } else if (sys == SYS_SBS) {
    switch (sigtype) {
    case 0:
      return CODE_L1C; /* L1C/A */
    case 6:
      return CODE_L5I; /* L5I (OEM6) */
    }
  }
  return 0;
}

static uint8_t obs2code(const char *obs) {
  int i;

  for (i = 1; *obscodes[i]; i++) {
    if (strcmp(obscodes[i], obs))
      continue;
    return (uint8_t)i;
  }
  return CODE_NONE;
}

static char *code2obs(uint8_t code) {
  if (code <= CODE_NONE || MAXCODE < code)
    return "";
  return obscodes[code];
}
static int code2freq_GPS(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '1':
    *freq = FREQ1;
    return 0; /* L1 */
  case '2':
    *freq = FREQ2;
    return 1; /* L2 */
  case '5':
    *freq = FREQ5;
    return 2; /* L5 */
  }
  return -1;
}

static int code2freq_GLO(uint8_t code, int fcn, double *freq) {
  char *obs = code2obs(code);

  if (fcn < -7 || fcn > 6)
    return -1;

  switch (obs[0]) {
  case '1':
    *freq = FREQ1_GLO + DFRQ1_GLO * fcn;
    return 0; /* G1 */
  case '2':
    *freq = FREQ2_GLO + DFRQ2_GLO * fcn;
    return 1; /* G2 */
  case '3':
    *freq = FREQ3_GLO;
    return 2; /* G3 */
  case '4':
    *freq = FREQ1a_GLO;
    return 0; /* G1a */
  case '6':
    *freq = FREQ2a_GLO;
    return 1; /* G2a */
  }
  return -1;
}

static int code2freq_GAL(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '1':
    *freq = FREQ1;
    return 0; /* E1 */
  case '7':
    *freq = FREQ7;
    return 1; /* E5b */
  case '5':
    *freq = FREQ5;
    return 2; /* E5a */
  case '6':
    *freq = FREQ6;
    return 3; /* E6 */
  case '8':
    *freq = FREQ8;
    return 4; /* E5ab */
  }
  return -1;
}
/* QZSS obs code to frequency ------------------------------------------------*/
static int code2freq_QZS(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '1':
    *freq = FREQ1;
    return 0; /* L1 */
  case '2':
    *freq = FREQ2;
    return 1; /* L2 */
  case '5':
    *freq = FREQ5;
    return 2; /* L5 */
  case '6':
    *freq = FREQ6;
    return 3; /* L6 */
  }
  return -1;
}
/* SBAS obs code to frequency ------------------------------------------------*/
static int code2freq_SBS(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '1':
    *freq = FREQ1;
    return 0; /* L1 */
  case '5':
    *freq = FREQ5;
    return 1; /* L5 */
  }
  return -1;
}
/* BDS obs code to frequency -------------------------------------------------*/
static int code2freq_BDS(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '1':
    *freq = FREQ1;
    return 0; /* B1C */
  case '2':
    *freq = FREQ1_CMP;
    return 0; /* B1I */
  case '7':
    *freq = FREQ2_CMP;
    return 1; /* B2I/B2b */
  case '5':
    *freq = FREQ5;
    return 2; /* B2a */
  case '6':
    *freq = FREQ3_CMP;
    return 3; /* B3 */
  case '8':
    *freq = FREQ8;
    return 4; /* B2ab */
  }
  return -1;
}
/* NavIC obs code to frequency -----------------------------------------------*/
static int code2freq_IRN(uint8_t code, double *freq) {
  char *obs = code2obs(code);

  switch (obs[0]) {
  case '5':
    *freq = FREQ5;
    return 0; /* L5 */
  case '9':
    *freq = FREQ9;
    return 1; /* S */
  }
  return -1;
}

static int code2idx(int sys, int code) {
  double freq;

  switch (sys) {
  case SYS_GPS:
    return code2freq_GPS(code, &freq);
  case SYS_GLO:
    return code2freq_GLO(code, 0, &freq);
  case SYS_GAL:
    return code2freq_GAL(code, &freq);
  case SYS_QZS:
    return code2freq_QZS(code, &freq);
  case SYS_SBS:
    return code2freq_SBS(code, &freq);
  case SYS_CMP:
    return code2freq_BDS(code, &freq);
  case SYS_IRN:
    return code2freq_IRN(code, &freq);
  }
  return -1;
}

static int decode_track_stat(uint32_t stat, int *sys, int *code, int *track,
                             int *plock, int *clock, int *parity, int *halfc) {
  int satsys, sigtype, idx = -1;

  *code = CODE_NONE;
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
    break; /* OEM6 */
  case 4:
    *sys = SYS_CMP;
    break; /* OEM6 F/W 6.400 */
  case 5:
    *sys = SYS_QZS;
    break; /* OEM6 */
  case 6:
    *sys = SYS_IRN;
    break; /* OEM7 */
  default:
    // trace(2, "oem4 unknown system: sys=%d\n", satsys);
    return -1;
  }
  if (!(*code = sig2code(*sys, sigtype)) || (idx = code2idx(*sys, *code)) < 0) {
    // trace(2, "oem4 signal type error: sys=%d sigtype=%d\n", *sys, sigtype);
    return -1;
  }
  return idx;
}

/* check code priority and return freq-index ---------------------------------*/
static int checkpri(const char *opt, int sys, int code, int idx) {
  int nex = NEXOBS;

  if (sys == SYS_GPS) {
    if (strstr(opt, "-GL1L") && idx == 0)
      return (code == CODE_L1L) ? 0 : -1;
    if (strstr(opt, "-GL2S") && idx == 1)
      return (code == CODE_L2X) ? 1 : -1;
    if (strstr(opt, "-GL2P") && idx == 1)
      return (code == CODE_L2P) ? 1 : -1;
    if (code == CODE_L1L)
      return (nex < 1) ? -1 : NFREQ;
    if (code == CODE_L2S)
      return (nex < 2) ? -1 : NFREQ + 1;
    if (code == CODE_L2P)
      return (nex < 3) ? -1 : NFREQ + 2;
  } else if (sys == SYS_GLO) {
    if (strstr(opt, "-RL2C") && idx == 1)
      return (code == CODE_L2C) ? 1 : -1;
    if (code == CODE_L2C)
      return (nex < 1) ? -1 : NFREQ;
  } else if (sys == SYS_GAL) {
    if (strstr(opt, "-EL6B") && idx == 3)
      return (code == CODE_L6B) ? 3 : -1;
    if (code == CODE_L6B)
      return (nex < 2) ? -1 : NFREQ;
  } else if (sys == SYS_QZS) {
    if (strstr(opt, "-JL1L") && idx == 0)
      return (code == CODE_L1L) ? 0 : -1;
    if (strstr(opt, "-JL1Z") && idx == 0)
      return (code == CODE_L1Z) ? 0 : -1;
    if (code == CODE_L1L)
      return (nex < 1) ? -1 : NFREQ;
    if (code == CODE_L1Z)
      return (nex < 2) ? -1 : NFREQ + 1;
  } else if (sys == SYS_CMP) {
    if (strstr(opt, "-CL1P") && idx == 0)
      return (code == CODE_L1P) ? 0 : -1;
    if (strstr(opt, "-CL7D") && idx == 0)
      return (code == CODE_L7D) ? 0 : -1;
    if (code == CODE_L1P)
      return (nex < 1) ? -1 : NFREQ;
    if (code == CODE_L7D)
      return (nex < 2) ? -1 : NFREQ + 1;
  }
  return idx < NFREQ ? idx : -1;
}

static int obsindex(std::vector<ObsPtr> &obs, gtime_t time, int sat) {
  int i, j;

  if (obs.size() >= MAXOBS)
    return -1;

  i = obs.size();

  obs.emplace_back(std::make_shared<Obs>());

  obs[i]->time = time;
  obs[i]->sat = sat;

  obs[i]->adr.resize(NFREQ + NEXOBS);
  obs[i]->psr.resize(NFREQ + NEXOBS);
  obs[i]->dopp.resize(NFREQ + NEXOBS);
  obs[i]->CN0.resize(NFREQ + NEXOBS);
  obs[i]->LLI.resize(NFREQ + NEXOBS);
  obs[i]->code.resize(NFREQ + NEXOBS);
  obs[i]->psr_std.resize(NFREQ + NEXOBS);
  obs[i]->dopp_std.resize(NFREQ + NEXOBS);
  obs[i]->adr_std.resize(NFREQ + NEXOBS);

  for (j = 0; j < NFREQ + NEXOBS; j++) {
    obs[i]->adr[j] = obs[i]->psr[j] = 0.0;
    obs[i]->dopp[j] = 0.0;
    obs[i]->CN0[j] = obs[i]->LLI[j] = 0;
    obs[i]->code[j] = CODE_NONE;
    obs[i]->psr_std[j] = obs[i]->dopp_std[j] = obs[i]->adr_std[j] = 0.0;
  }
  return i;
}

static int decode_rangecmpb(const unsigned char *raw, size_t len,
                            std::vector<ObsPtr> &obs, gtime_t time,
                            double glo_bias = 0.0) {
  unsigned char *p = const_cast<unsigned char *>(raw);
  char *q;
  double psr, adr, adr_rolls, lockt, tt, dop, snr, freq = 0.0;
  int i, index, nobs, prn, sat, sys, code, idx, track, plock, clock, parity,
      halfc, lli;

  nobs = U4(p);
  if (len < 4 + nobs * 24) {
    // trace(2, "oem4 rangecmpb length error: len=%d nobs=%d\n", raw->len,
    // nobs);
    return -1;
  }

  for (i = 0, p += 4; i < nobs; i++, p += 24) {
    if ((idx = decode_track_stat(U4(p), &sys, &code, &track, &plock, &clock,
                                 &parity, &halfc)) < 0) {
      continue;
    }
    prn = U1(p + 17);
    if (sys == SYS_GLO)
      prn -= 37;
    if (sys == SYS_SBS && prn >= MINPRNQZS_S && prn <= MAXPRNQZS_S &&
        code == CODE_L1C) {
      sys = SYS_QZS;
      prn += 10;
      code = CODE_L1Z; /* QZS L1S */
    }
    if (!(sat = sat_no(sys, prn))) {
      // trace(3, "oem4 rangecmpb satellite number error: sys=%d,prn=%d\n", sys,
      //      prn);
      continue;
    }
    if (sys == SYS_GLO && !parity)
      continue;

    dop = exsign(U4(p + 4) & 0xFFFFFFF, 28) / 256.0;
    psr = (U4(p + 7) >> 4) / 128.0 + U1(p + 11) * 2097152.0;

    double psr_std = 0.;

    switch ((*(p + 16)) & 0x0F) {
    case 0:
      psr_std = 0.050f;
      break;
    case 1:
      psr_std = 0.075f;
      break;
    case 2:
      psr_std = 0.113f;
      break;
    case 3:
      psr_std = 0.169f;
      break;
    case 4:
      psr_std = 0.253f;
      break;
    case 5:
      psr_std = 0.380f;
      break;
    case 6:
      psr_std = 0.570f;
      break;
    case 7:
      psr_std = 0.854f;
      break;
    case 8:
      psr_std = 1.281f;
      break;
    case 9:
      psr_std = 2.375f;
      break;
    case 10:
      psr_std = 4.750f;
      break;
    case 11:
      psr_std = 9.500f;
      break;
    case 12:
      psr_std = 19.000f;
      break;
    case 13:
      psr_std = 38.000f;
      break;
    case 14:
      psr_std = 76.000f;
      break;
    case 15:
      psr_std = 152.000f;
      break;
    };

    uint8_t stdev_adr = (*(p + 16)) >> 4;

    if (true) {
      adr = I4(p + 12) / 256.0;
      adr_rolls = (psr * freq / CLIGHT + adr) / MAXVAL;
      adr = -adr + MAXVAL * floor(adr_rolls + (adr_rolls <= 0 ? -0.5 : 0.5));
      if (sys == SYS_GLO)
        adr += glo_bias * freq / CLIGHT;
    } else {
      adr = 1e-9;
    }

    lockt = (U4(p + 18) & 0x1FFFFF) / 32.0;

    lli = 0;
    if (!parity)
      lli |= LLI_HALFC;
    if (halfc)
      lli |= LLI_HALFA;

    snr = ((U2(p + 20) & 0x3FF) >> 5) + 20.0;
    if (!clock)
      psr = 0.0; /* code unlock */
    if (!plock)
      adr = dop = 0.0; /* phase unlock */

    if ((index = obsindex(obs, time, sat)) >= 0) {
      obs[index]->adr[idx] = adr;
      obs[index]->psr[idx] = psr;
      obs[index]->dopp[idx] = (float)dop;
      obs[index]->CN0[idx] = (uint16_t)(snr / SNR_UNIT + 0.5);
      obs[index]->LLI[idx] = (uint8_t)lli;
      obs[index]->psr_std[idx] = psr_std;
      obs[index]->adr_std[idx] = (float)(stdev_adr + 1) / 512;
    }
  }
  return 1;
}

bynav_gps_msgs::GnssMeasMsgPtr
RangrcmpbParser::ParseBinary(const BinaryMessage &bin_msg) {
  
  bynav_gps_msgs::GnssMeasMsgPtr ros_msg =
      boost::make_shared<bynav_gps_msgs::GnssMeasMsg>();

  bynav_gps_msgs::BynavMessageHeader bynav_msg_header;
  HeaderParser header_parser;
  bynav_msg_header = header_parser.ParseBinary(bin_msg);
  bynav_msg_header.message_name = MESSAGE_NAME;

  gtime_t time =
      gpst2time(bynav_msg_header.gps_week_num, bynav_msg_header.gps_seconds);

  std::vector<ObsPtr> meas;
  int ret =
      decode_rangecmpb(bin_msg.data_.data(), bin_msg.data_.size(), meas, time);
  if (ret < 0) {
    std::stringstream error;
    error << "Unexpected BDSEPHEMERISB message length: "
          << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  *ros_msg = meas2msg(meas);
  ros_msg->bynav_msg_header = bynav_msg_header;
  return ros_msg;
}

} // namespace bynav_gps_driver