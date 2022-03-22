#ifndef BYNAV_RAWROS_H
#define BYNAV_RAWROS_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <time.h>
#include <vector>

#include <bynav_gps_msgs/GnssEphemMsg.h>
#include <bynav_gps_msgs/GnssGloEphemMsg.h>
#include <bynav_gps_msgs/GnssMeasMsg.h>
#include <bynav_gps_msgs/GnssObsMsg.h>
#include <bynav_gps_msgs/GnssTimeMsg.h>
#include <ros/ros.h>

#include <bynav_gps_driver/parsers/raw.h>

namespace bynav_gps_driver {

bynav_gps_msgs::GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr);

EphemPtr msg2ephem(const bynav_gps_msgs::GnssEphemMsgConstPtr &gnss_ephem_msg);

bynav_gps_msgs::GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr);

GloEphemPtr msg2glo_ephem(
    const bynav_gps_msgs::GnssGloEphemMsgConstPtr &gnss_glo_ephem_msg);

bynav_gps_msgs::GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas);

std::vector<ObsPtr>
msg2meas(const bynav_gps_msgs::GnssMeasMsgConstPtr &gnss_meas_msg);

} // namespace bynav_gps_driver

#endif // BYNAV_RAWROS_H
