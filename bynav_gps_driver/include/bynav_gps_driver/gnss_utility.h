#ifndef GNSS_UTILITY_HPP_
#define GNSS_UTILITY_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <bynav_gps_driver/gnss_constant.h>

namespace bynav_gps_driver {

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

Eigen::Vector3d geo2ecef(const Eigen::Vector3d &lla);

Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz);

double Kepler(const double mk, const double es);

double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr);

Eigen::Vector3d eph2pos(const gtime_t &curr_time, const EphemPtr ephem_ptr,
                        double *svdt);

Eigen::Vector3d eph2vel(const gtime_t &curr_time, const EphemPtr ephem,
                        double *svddt);

void deq(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
         const Eigen::Vector3d &acc, Eigen::Vector3d &pos_dot,
         Eigen::Vector3d &vel_dot);

void glo_orbit(double dt, Eigen::Vector3d &pos, Eigen::Vector3d &vel,
               const Eigen::Vector3d &acc);

double geph2svdt(const gtime_t &curr_time, const GloEphemPtr geph_ptr);

Eigen::Vector3d geph2pos(const gtime_t &curr_time, const GloEphemPtr geph_ptr,
                         double *svdt);

Eigen::Vector3d geph2vel(const gtime_t &curr_time, const GloEphemPtr geph_ptr,
                         double *svddt);

void sat_azel(const Eigen::Vector3d &rev_pos, const Eigen::Vector3d &sat_pos,
              double *azel);

Eigen::Vector3d ecef2enu(const Eigen::Vector3d &pos_lla,
                         const Eigen::Vector3d &v_ecef);

Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo);

Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef);

double calculate_trop_delay(gtime_t time, const Eigen::Vector3d &rev_lla,
                            const double *azel);

double calculate_ion_delay(gtime_t t, const std::vector<double> &ion_parameters,
                           const Eigen::Vector3d &rev_lla, const double *azel);

std::string sat2str(uint32_t sat_no);

uint32_t str2sat(const std::string &sat_str);

double L1_freq(const ObsPtr &obs, int *l1_idx);

std::string exec(const std::string &cmd);

} // namespace bynav_gps_driver

#endif
