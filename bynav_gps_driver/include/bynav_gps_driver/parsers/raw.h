#ifndef BYNAV_RAW_H
#define BYNAV_RAW_H

#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#define MIN_PRN_GPS 1  /* min satellite PRN number of GPS */
#define MAX_PRN_GPS 32 /* max satellite PRN number of GPS */
#define N_SAT_GPS                                                              \
  (MAX_PRN_GPS - MIN_PRN_GPS + 1) /* number of GPS satellites                  \
                                   */
#define N_SYS_GPS 1

#define MIN_PRN_GLO 1  /* min satellite slot number of GLONASS */
#define MAX_PRN_GLO 27 /* max satellite slot number of GLONASS */
#define N_SAT_GLO                                                              \
  (MAX_PRN_GLO - MIN_PRN_GLO + 1) /* number of GLONASS satellites */
#define N_SYS_GLO 1

#define MIN_PRN_GAL 1  /* min satellite PRN number of Galileo */
#define MAX_PRN_GAL 38 /* max satellite PRN number of Galileo */
#define N_SAT_GAL                                                              \
  (MAX_PRN_GAL - MIN_PRN_GAL + 1) /* number of Galileo satellites */
#define N_SYS_GAL 1

#define MIN_PRN_BDS 1  /* min satellite sat number of BeiDou */
#define MAX_PRN_BDS 63 /* max satellite sat number of BeiDou */
#define N_SAT_BDS                                                              \
  (MAX_PRN_BDS - MIN_PRN_BDS + 1) /* number of BeiDou satellites */
#define N_SYS_BDS 1

#define N_SYS                                                                  \
  (N_SYS_GPS + N_SYS_GLO + N_SYS_GAL++ N_SYS_BDS) /* number of systems */

#define MAX_SAT (N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + N_SAT_BDS)

#ifndef MAXOBS
#define MAXOBS 64 /* max number of obs in an epoch */
#endif

#ifndef NFREQ
#define NFREQ 3 /* number of carrier frequencies */
#endif
#define NFREQGLO 2 /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS 0 /* number of extended obs codes */
#endif

#define CODE_NONE 0 /* obs code: none or unknown */
#define CODE_L1C 1  /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P 2  /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W 3  /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y 4  /* obs code: L1Y        (GPS) */
#define CODE_L1M 5  /* obs code: L1M        (GPS) */
#define CODE_L1N 6  /* obs code: L1codeless (GPS) */
#define CODE_L1S 7  /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L 8  /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E 9  /* (not used) */
#define CODE_L1A 10 /* obs code: E1A        (GAL) */
#define CODE_L1B 11 /* obs code: E1B        (GAL) */
#define CODE_L1X 12 /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z 13 /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C 14 /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D 15 /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S 16 /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L 17 /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X 18 /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P 19 /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W 20 /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y 21 /* obs code: L2Y        (GPS) */
#define CODE_L2M 22 /* obs code: L2M        (GPS) */
#define CODE_L2N 23 /* obs code: L2codeless (GPS) */
#define CODE_L5I 24 /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q 25 /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X 26 /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) */
#define CODE_L7I 27 /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q 28 /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X 29 /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A 30 /* obs code: E6A        (GAL) */
#define CODE_L6B 31 /* obs code: E6B        (GAL) */
#define CODE_L6C 32 /* obs code: E6C        (GAL) */
#define CODE_L6X 33 /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z 34 /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S 35 /* obs code: LEXS       (QZS) */
#define CODE_L6L 36 /* obs code: LEXL       (QZS) */
#define CODE_L8I 37 /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q 38 /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X 39 /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I 40 /* obs code: B1I        (BDS) */
#define CODE_L2Q 41 /* obs code: B1Q        (BDS) */
#define CODE_L6I 42 /* obs code: B3I        (BDS) */
#define CODE_L6Q 43 /* obs code: B3Q        (BDS) */
#define CODE_L3I 44 /* obs code: G3I        (GLO) */
#define CODE_L3Q 45 /* obs code: G3Q        (GLO) */
#define CODE_L3X 46 /* obs code: G3I+Q      (GLO) */
#define CODE_L1I 47 /* obs code: B1I        (BDS) */
#define CODE_L1Q 48 /* obs code: B1Q        (BDS) */
#define CODE_L5A 49 /* obs code: L5A SPS    (IRN) */
#define CODE_L5B 50 /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C 51 /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A 52 /* obs code: SA SPS     (IRN) */
#define CODE_L9B 53 /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C 54 /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X 55 /* obs code: SB+C       (IRN) */
#define MAXCODE 55  /* max number of obs code */

namespace bynav_gps_driver {

const int SYS_NONE = 0x00; //!<   navigation system: none
const int SYS_GPS = 0x01;  //!<   navigation system: GPS
const int SYS_SBS = 0x02;  //!<   navigation system: SBAS
const int SYS_GLO = 0x04;  //!<   navigation system: GLONASS
const int SYS_GAL = 0x08;  //!<   navigation system: Galileo
const int SYS_QZS = 0x10;  //!<   navigation system: QZSS
const int SYS_BDS = 0x20;  //!<   navigation system: BeiDou
const int SYS_IRN = 0x40;  //!<   navigation system: IRNS
const int SYS_LEO = 0x80;  //!<   navigation system: LEO
const int SYS_ALL = 0xFF;  //!<   navigation system: all
const int SYS_CMP = 0x20   /* navigation system: BeiDou */

#define U1(p) (*((unsigned char *)(p)))

    static inline unsigned short
    U2(unsigned char *p) {
  unsigned short u;
  memcpy(&u, p, 2);
  return u;
}
static inline unsigned int U4(unsigned char *p) {
  unsigned int u;
  memcpy(&u, p, 4);
  return u;
}
static inline int I4(unsigned char *p) {
  int i;
  memcpy(&i, p, 4);
  return i;
}
static inline float R4(unsigned char *p) {
  float r;
  memcpy(&r, p, 4);
  return r;
}
static inline double R8(unsigned char *p) {
  double r;
  memcpy(&r, p, 8);
  return r;
}

struct gtime_t {
  time_t time; /* time (s) expressed by standard time_t */
  double sec;  /* fraction of second under 1 s */
};

uint32_t sat_no(uint32_t sys, uint32_t prn);

uint32_t satsys(uint32_t sat, uint32_t *prn);

gtime_t epoch2time(const double *ep);

void time2epoch(gtime_t t, double *ep);

gtime_t gpst2time(uint32_t week, double tow);

double time2gpst(gtime_t t, uint32_t *week);

gtime_t gst2time(int week, double tow);

double time2gst(gtime_t t, int *week);

gtime_t bdt2time(int week, double tow);

double time2bdt(gtime_t t, int *week);

gtime_t gpst2utc(gtime_t t);

gtime_t utc2gpst(gtime_t t);

double julian_day(std::vector<double> datetime);

uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime);

double time2doy(gtime_t time);

double time_diff(gtime_t t1, gtime_t t2);

gtime_t time_add(gtime_t t, double sec);

double time2sec(gtime_t time);

gtime_t sec2time(const double sec);

static inline int exsign(unsigned int v, int bits) {
  return (int)(v & (1 << (bits - 1)) ? v | (~0u << bits) : v);
}

static inline unsigned char chksum(const unsigned char *buff, int len) {
  unsigned char sum = 0;
  int i;
  for (i = 0; i < len; i++)
    sum ^= buff[i];
  return sum;
}

static inline gtime_t adjweek(gtime_t time, double tow) {
  double tow_p;
  uint32_t week;
  tow_p = time2gpst(time, &week);
  if (tow < tow_p - 302400.0)
    tow += 604800.0;
  else if (tow > tow_p + 302400.0)
    tow -= 604800.0;
  return gpst2time(week, tow);
}

static inline int uraindex(double value) {
  static const double ura_eph[] = {2.4,    3.4,    4.85,   6.85,  9.65,  13.65,
                                   24.0,   48.0,   96.0,   192.0, 384.0, 768.0,
                                   1536.0, 3072.0, 6144.0, 0.0};
  int i;
  for (i = 0; i < 15; i++)
    if (ura_eph[i] >= value)
      break;
  return i;
}

struct EphemBase {
  virtual ~EphemBase() = default;
  uint32_t sat;    /* satellite number */
  gtime_t ttr;     /* transmission time in GPST */
  gtime_t toe;     /* ephemeris reference time in GPST */
  uint32_t health; /* satellite health */
  double ura;      /* satellite signal accuracy */
  uint32_t iode;
};
typedef std::shared_ptr<EphemBase> EphemBasePtr;

struct GloEphem : EphemBase {
  int freqo;           /* satellite frequency number */
  uint32_t age;        /* satellite age */
  double pos[3];       /* satellite position (ecef) (m) */
  double vel[3];       /* satellite velocity (ecef) (m/s) */
  double acc[3];       /* satellite acceleration (ecef) (m/s^2) */
  double tau_n, gamma; /* SV clock bias (s)/relative freq bias */
  double delta_tau_n;  /* delay between L1 and L2 (s) */
};
typedef std::shared_ptr<GloEphem> GloEphemPtr;

struct Ephem : EphemBase {
  gtime_t toc;    /* clock correction reference time in GPST */
  double toe_tow; /* toe seconds within the week */
  uint32_t week;
  uint32_t iodc;
  uint32_t code;
  double A, e, i0, omg, OMG0, M0, delta_n, OMG_dot,
      i_dot; /* SV orbit parameters */
  double cuc, cus, crc, crs, cic, cis;
  double af0, af1, af2; /* SV clock parameters */
  double tgd[2];        /* group delay parameters */
                        /* GPS    :tgd[0]=TGD */
                        /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                        /* BDS    :tgd[0]=BGD1,tgd[1]=BGD2 */
  double A_dot, n_dot;  /* Adot,ndot for CNAV */
};
typedef std::shared_ptr<Ephem> EphemPtr;

struct Obs /* observation data record */
{
  gtime_t time;                 /* receiver sampling time (GPST) */
  uint32_t sat;                 /* satellite number */
  std::vector<double> freqs;    /* received satellite frequencies */
  std::vector<double> CN0;      /* signal strength */
  std::vector<uint8_t> LLI;     /* signal lost-lock indictor */
  std::vector<uint8_t> code;    /* code indicator (CODE_???) */
  std::vector<double> psr;      /* observation data pseudorange (m) */
  std::vector<double> psr_std;  /* pseudorange std (m) */
  std::vector<double> cp;       /* observation data carrier-phase (cycle) */
  std::vector<double> cp_std;   /* carrier-phase std (cycle) */
  std::vector<double> dopp;     /* observation data doppler frequency (Hz) */
  std::vector<double> dopp_std; /* doppler std (Hz) */
  std::vector<uint8_t> status;  /* cycle slip valid flag. bit_0 (psr valid),
                                   bit_1(cp valid), bit_2(half cp valid, *ublox),
                                   bit_3(half cp subtracted, *ublox) */
};
typedef std::shared_ptr<Obs> ObsPtr;

} // namespace bynav_gps_driver

#endif // BYNAV_RAW_H
