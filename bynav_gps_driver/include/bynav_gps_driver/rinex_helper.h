#ifndef RINEX_HELPER_HPP_
#define RINEX_HELPER_HPP_

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <time.h>
#include <vector>

#include <bynav_gps_driver/gnss_constant.h>
#include <bynav_gps_driver/gnss_utility.h>

namespace bynav_gps_driver {

void rinex2ephems(const std::string &rinex_filepath,
                  std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem);

void rinex2obs(const std::string &rinex_filepath,
               std::vector<std::vector<ObsPtr>> &rinex_meas);

void obs2rinex(const std::string &rinex_filepath,
               const std::vector<std::vector<ObsPtr>> &gnss_meas);

void rinex2iono_params(const std::string &rinex_filepath,
                       std::vector<double> &iono_params);

} // namespace bynav_gps_driver

#endif