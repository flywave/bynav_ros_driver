#include <bynav_gps_driver/parsers/raw.h>

#include <cmath>

namespace bynav_gps_driver {

const double leaps[MAXLEAPS + 1][7] = {/* leap seconds (y,m,d,h,m,s,utc-gpst) */
                                       {2017, 1, 1, 0, 0, 0, -18},
                                       {2015, 7, 1, 0, 0, 0, -17},
                                       {2012, 7, 1, 0, 0, 0, -16},
                                       {2009, 1, 1, 0, 0, 0, -15},
                                       {2006, 1, 1, 0, 0, 0, -14},
                                       {1999, 1, 1, 0, 0, 0, -13},
                                       {1997, 7, 1, 0, 0, 0, -12},
                                       {1996, 1, 1, 0, 0, 0, -11},
                                       {1994, 7, 1, 0, 0, 0, -10},
                                       {1993, 7, 1, 0, 0, 0, -9},
                                       {1992, 7, 1, 0, 0, 0, -8},
                                       {1991, 1, 1, 0, 0, 0, -7},
                                       {1990, 1, 1, 0, 0, 0, -6},
                                       {1988, 1, 1, 0, 0, 0, -5},
                                       {1985, 7, 1, 0, 0, 0, -4},
                                       {1983, 7, 1, 0, 0, 0, -3},
                                       {1982, 7, 1, 0, 0, 0, -2},
                                       {1981, 7, 1, 0, 0, 0, -1},
                                       {0}};

const static double gpst0[] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */
const static double gst0[] = {1999, 8, 22,
                              0,    0, 0}; /* galileo system time reference */
const static double bdt0[] = {2006, 1, 1, 0, 0, 0}; /* beidou time reference */

uint32_t sat_no(uint32_t sys, uint32_t prn) {
  if (prn == 0)
    return 0;
  switch (sys) {
  case SYS_GPS:
    if (prn < MIN_PRN_GPS || prn > MAX_PRN_GPS)
      return 0;
    return prn - MIN_PRN_GPS + 1;
  case SYS_GLO:
    if (prn < MIN_PRN_GLO || prn > MAX_PRN_GLO)
      return 0;
    return N_SAT_GPS + prn - MIN_PRN_GLO + 1;
  case SYS_GAL:
    if (prn < MIN_PRN_GAL || prn > MAX_PRN_GAL)
      return 0;
    return N_SAT_GPS + N_SAT_GLO + prn - MIN_PRN_GAL + 1;
  case SYS_BDS:
    if (prn < MIN_PRN_BDS || prn > MAX_PRN_BDS)
      return 0;
    return N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + prn - MIN_PRN_BDS + 1;
  }
  return 0;
}

uint32_t satsys(uint32_t sat, uint32_t *prn) {
  uint32_t sys = SYS_NONE;
  if (sat <= 0 || sat > MAX_SAT)
    sat = 0;
  else if (sat <= N_SAT_GPS) {
    sys = SYS_GPS;
    sat += MIN_PRN_GPS - 1;
  } else if ((sat -= N_SAT_GPS) <= N_SAT_GLO) {
    sys = SYS_GLO;
    sat += MIN_PRN_GLO - 1;
  } else if ((sat -= N_SAT_GLO) <= N_SAT_GAL) {
    sys = SYS_GAL;
    sat += MIN_PRN_GAL - 1;
  } else if ((sat -= N_SAT_GAL) <= N_SAT_BDS) {
    sys = SYS_BDS;
    sat += MIN_PRN_BDS - 1;
  } else
    sat = 0;
  if (prn)
    *prn = sat;
  return sys;
}

gtime_t epoch2time(const double *ep) {
  const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
  gtime_t time = {0};
  int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

  if (year < 1970 || year > 2099 || mon < 1 || mon > 12)
    return time;

  /* leap year if year%4==0 in 1901-2099 */
  days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 +
         (year % 4 == 0 && mon >= 3 ? 1 : 0);
  sec = (int)std::floor(ep[5]);
  time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
  time.sec = ep[5] - sec;
  return time;
}

void time2epoch(gtime_t t, double *ep) {
  const int mday[] = {/* # of days in a month */
                      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                      31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int days, sec, mon, day;

  /* leap year if year%4==0 in 1901-2099 */
  days = (int)(t.time / 86400);
  sec = (int)(t.time - (time_t)days * 86400);
  for (day = days % 1461, mon = 0; mon < 48; mon++) {
    if (day >= mday[mon])
      day -= mday[mon];
    else
      break;
  }
  ep[0] = 1970 + days / 1461 * 4 + mon / 12;
  ep[1] = mon % 12 + 1;
  ep[2] = day + 1;
  ep[3] = sec / 3600;
  ep[4] = sec % 3600 / 60;
  ep[5] = sec % 60 + t.sec;
}

gtime_t gpst2time(uint32_t week, double tow) {
  gtime_t t = epoch2time(gpst0);
  if (tow < -1E9 || tow > 1E9)
    tow = 0.0;
  t.time += 86400 * 7 * week + (int)tow;
  t.sec = tow - (int)tow;
  return t;
}

double time2gpst(gtime_t t, uint32_t *week) {
  gtime_t t0 = epoch2time(gpst0);
  time_t sec = t.time - t0.time;
  uint32_t w = (uint32_t)(sec / (86400 * 7));

  if (week)
    *week = w;
  return (double)(sec - w * 86400 * 7) + t.sec;
}

gtime_t gst2time(int week, double tow) {
  gtime_t t = epoch2time(gst0);

  if (tow < -1E9 || tow > 1E9)
    tow = 0.0;
  t.time += 86400 * 7 * week + (int)tow;
  t.sec = tow - (int)tow;
  return t;
}

double time2gst(gtime_t t, int *week) {
  gtime_t t0 = epoch2time(gst0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week)
    *week = w;
  return (double)(sec - w * 86400 * 7) + t.sec;
}

gtime_t bdt2time(int week, double tow) {
  gtime_t t = epoch2time(bdt0);

  if (tow < -1E9 || tow > 1E9)
    tow = 0.0;
  t.time += 86400 * 7 * week + (int)tow;
  t.sec = tow - (int)tow;
  return t;
}

double time2bdt(gtime_t t, int *week) {
  gtime_t t0 = epoch2time(bdt0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week)
    *week = w;
  return (double)(sec - w * 86400 * 7) + t.sec;
}

gtime_t gpst2utc(gtime_t t) {
  gtime_t tu;
  int i;

  for (i = 0; leaps[i][0] > 0; i++) {
    tu = time_add(t, leaps[i][6]);
    if (time_diff(tu, epoch2time(leaps[i])) >= 0.0)
      return tu;
  }
  return t;
}

gtime_t utc2gpst(gtime_t t) {
  int i;

  for (i = 0; leaps[i][0] > 0; i++) {
    if (time_diff(t, epoch2time(leaps[i])) >= 0.0)
      return time_add(t, -leaps[i][6]);
  }
  return t;
}

double julian_day(std::vector<double> datetime) {
  // LOG_IF(FATAL, datetime.size() != 6) << "datetime should contain 6 fields";
  // LOG_IF(FATAL, datetime[0] < 1901 || datetime[0] > 2099)
  //    << datetime[0] << " should within range 1900 ~ 2100";

  if (datetime[1] <= 2) {
    datetime[1] += 12;
    datetime[0] -= 1;
  }
  double julian_day = floor(365.25 * datetime[0]) +
                      floor(30.6001 * (datetime[1] + 1)) - 15 + 1720996.5 +
                      datetime[2] + datetime[3] / 24 + datetime[4] / 60 / 24 +
                      datetime[5] / 3600 / 24;
  return julian_day;
}

uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime) {
  gtime_t given_t = epoch2time(&(datetime[0]));
  for (size_t i = 0; leaps[i][0] > 0; ++i) {
    gtime_t tu = time_add(given_t, leaps[i][6]);
    if (time_diff(tu, epoch2time(leaps[i])) >= 0.0)
      return static_cast<uint32_t>(-leaps[i][6]);
  }
  return static_cast<uint32_t>(-1);
}

double time2doy(gtime_t time) {
  double ep[6];
  time2epoch(time, ep);
  ep[1] = ep[2] = 1.0;
  ep[3] = ep[4] = ep[5] = 0.0;
  return time_diff(time, epoch2time(ep)) / 86400.0 + 1.0;
}

double time_diff(gtime_t t1, gtime_t t2) {
  return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

gtime_t time_add(gtime_t t, double sec) {
  t.sec += sec;
  double tt = floor(t.sec);
  t.time += static_cast<int>(tt);
  t.sec -= tt;
  return t;
}

double time2sec(gtime_t time) {
  return static_cast<double>(time.time) + time.sec;
}

gtime_t sec2time(const double sec) {
  gtime_t time;
  time.time = floor(sec);
  time.sec = sec - time.time;
  return time;
}

} // namespace bynav_gps_driver