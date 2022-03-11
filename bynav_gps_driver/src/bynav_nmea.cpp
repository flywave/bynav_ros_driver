#include <bynav_gps_driver/bynav_nmea.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sstream>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

namespace bynav_gps_driver {

BynavNmea::BynavNmea()
    : gpsfix_sync_tol_(0.01), wait_for_sync_(true), imu_rate_forced_(false),
      utc_offset_(0), corrimudata_msgs_(MAX_BUFFER_SIZE),
      gpgga_msgs_(MAX_BUFFER_SIZE), gpgsv_msgs_(MAX_BUFFER_SIZE),
      gphdt_msgs_(MAX_BUFFER_SIZE), gprmc_msgs_(MAX_BUFFER_SIZE),
      imu_msgs_(MAX_BUFFER_SIZE), inspva_msgs_(MAX_BUFFER_SIZE),
      inspvax_msgs_(MAX_BUFFER_SIZE), insstdev_msgs_(MAX_BUFFER_SIZE),
      bynav_positions_(MAX_BUFFER_SIZE), bynav_pjk_positions_(MAX_BUFFER_SIZE),
      bynav_velocities_(MAX_BUFFER_SIZE),
      bestpos_sync_buffer_(SYNC_BUFFER_SIZE),
      bestvel_sync_buffer_(SYNC_BUFFER_SIZE), heading_msgs_(MAX_BUFFER_SIZE),
      gpdop_msgs_(MAX_BUFFER_SIZE), time_msgs_(MAX_BUFFER_SIZE),
      imu_rate_(-1.0), enable_imu_(false) {}

bool BynavNmea::Connect(const std::string &device, ConnectionType connection,
                        BynavMessageOpts const &oopts) {
  BynavMessageOpts opts = oopts;
  opts["gpgga"] = 0.05;
  opts["gprmc"] = 0.05;
  opts["bestposa"] = 0.05;
  opts["gpzda"] = 1.0;
  opts["gpgsv"] = 1;
  return BynavControl::Connect(device, connection, opts);
}

BynavNmea::ReadResult BynavNmea::ProcessData() {
  BynavNmea::ReadResult read_result = ReadData();

  if (read_result != READ_SUCCESS) {
    return read_result;
  }

  ros::Time stamp = ros::Time::now();
  std::vector<NmeaSentence> nmea_sentences;
  std::vector<BynavSentence> bynav_sentences;
  std::vector<BinaryMessage> binary_messages;

  if (!data_buffer_.empty()) {
    nmea_buffer_.insert(nmea_buffer_.end(), data_buffer_.begin(),
                        data_buffer_.end());

    data_buffer_.clear();

    std::string remaining_buffer;

    if (!extractor_.ExtractCompleteMessages(nmea_buffer_, nmea_sentences,
                                            bynav_sentences, binary_messages,
                                            remaining_buffer)) {
      read_result = READ_PARSE_FAILED;
      error_msg_ = "Parse failure extracting sentences.";
    }

    nmea_buffer_ = remaining_buffer;

    ROS_DEBUG("Parsed: %lu NMEA / %lu Bynav / %lu Binary messages",
              nmea_sentences.size(), bynav_sentences.size(),
              binary_messages.size());
    if (!nmea_buffer_.empty()) {
      ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
    }
  }

  double most_recent_utc_time = extractor_.GetMostRecentUtcTime(nmea_sentences);

  for (const auto &sentence : nmea_sentences) {
    try {
      BynavNmea::ReadResult result =
          ParseNmeaSentence(sentence, stamp, most_recent_utc_time);
      if (result != READ_SUCCESS) {
        read_result = result;
      }
    } catch (const ParseException &p) {
      error_msg_ = p.what();
      ROS_WARN("%s", p.what());
      ROS_WARN("For sentence: [%s]",
               boost::algorithm::join(sentence.body, ",").c_str());
      read_result = READ_PARSE_FAILED;
    }
  }

  for (const auto &sentence : bynav_sentences) {
    try {
      BynavNmea::ReadResult result = ParseBynavSentence(sentence, stamp);
      if (result != READ_SUCCESS) {
        read_result = result;
      }
    } catch (const ParseException &p) {
      error_msg_ = p.what();
      ROS_WARN("%s", p.what());
      read_result = READ_PARSE_FAILED;
    }
  }

  for (const auto &msg : binary_messages) {
    try {
      BynavNmea::ReadResult result = ParseBinaryMessage(msg, stamp);
      if (result != READ_SUCCESS) {
        read_result = result;
      }
    } catch (const ParseException &p) {
      error_msg_ = p.what();
      ROS_WARN("%s", p.what());
      read_result = READ_PARSE_FAILED;
    }
  }

  return read_result;
}

void BynavNmea::GetBynavPositions(
    std::vector<bynav_gps_msgs::BynavPositionPtr> &positions) {
  positions.clear();
  positions.insert(positions.end(), bynav_positions_.begin(),
                   bynav_positions_.end());
  bynav_positions_.clear();
}

void BynavNmea::GetBynavPJKPositions(
    std::vector<bynav_gps_msgs::PtnlPJKPtr> &positions) {
  positions.clear();
  positions.insert(positions.end(), bynav_pjk_positions_.begin(),
                   bynav_pjk_positions_.end());
  bynav_pjk_positions_.clear();
}

void BynavNmea::GetBynavVelocities(
    std::vector<bynav_gps_msgs::BynavVelocityPtr> &velocities) {
  velocities.resize(bynav_velocities_.size());
  std::copy(bynav_velocities_.begin(), bynav_velocities_.end(),
            velocities.begin());
  bynav_velocities_.clear();
}

void BynavNmea::GetFixMessages(
    std::vector<gps_common::GPSFixPtr> &fix_messages) {
  fix_messages.clear();

  while (!bestpos_sync_buffer_.empty()) {
    auto &bestpos = bestpos_sync_buffer_.front();

    bool synced = false;

    auto gpsFix = boost::make_shared<gps_common::GPSFix>();

    while (!bestvel_sync_buffer_.empty()) {
      auto &bestvel = bestvel_sync_buffer_.front();

      double time_diff = std::fabs(bestvel->bynav_msg_header.gps_seconds -
                                   bestpos->bynav_msg_header.gps_seconds);
      if (time_diff < gpsfix_sync_tol_) {
        gpsFix->track = bestvel->track_gnd;
        gpsFix->speed = std::sqrt(std::pow(bestvel->horizontal_speed, 2) +
                                  std::pow(bestvel->vertical_speed, 2));
        synced = true;
        break;
      } else if (bestvel->bynav_msg_header.gps_seconds <
                 bestpos->bynav_msg_header.gps_seconds) {
        bestvel_sync_buffer_.pop_front();
      } else {
        break;
      }
    }

    if (!synced && wait_for_sync_) {
      break;
    }

    gpsFix->header.stamp = ros::Time::now();
    gpsFix->altitude = bestpos->height;
    gpsFix->latitude = bestpos->lat;
    gpsFix->longitude = bestpos->lon;

    if (bestpos->solution_status == "SOL_COMPUTED") {
      gpsFix->status.status = gps_common::GPSStatus::STATUS_FIX;
    } else {
      gpsFix->status.status = gps_common::GPSStatus::STATUS_NO_FIX;
    }

    gpsFix->time = bestpos->bynav_msg_header.gps_seconds;

    gpsFix->status.header.stamp = gpsFix->header.stamp;
    gpsFix->status.satellites_visible = bestpos->num_satellites_tracked;
    gpsFix->status.satellites_used = bestpos->num_satellites_used_in_solution;

    double sigma_x = bestpos->lon_sigma;
    double sigma_x_squared = sigma_x * sigma_x;
    gpsFix->position_covariance[0] = sigma_x_squared;

    double sigma_y = bestpos->lat_sigma;
    double sigma_y_squared = sigma_y * sigma_y;
    gpsFix->position_covariance[4] = sigma_y_squared;

    double sigma_z = bestpos->height_sigma;
    gpsFix->position_covariance[8] = sigma_z * sigma_z;

    gpsFix->err_horz = 2.0 * std::sqrt(sigma_x_squared + sigma_y_squared);

    gpsFix->err = 0.833 * (sigma_x + sigma_y + sigma_z);

    gpsFix->err_vert = 2.0 * sigma_z;

    gpsFix->position_covariance_type =
        gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    if (latest_gpdop_) {
      gpsFix->gdop = latest_gpdop_->gdop;
      gpsFix->pdop = latest_gpdop_->pdop;
      gpsFix->hdop = latest_gpdop_->hdop;
      gpsFix->vdop = latest_gpdop_->vdop;
    }

    fix_messages.push_back(gpsFix);
    bestpos_sync_buffer_.pop_front();
  }
}

void BynavNmea::GetHeadingMessages(
    std::vector<bynav_gps_msgs::HeadingPtr> &headings) {
  headings.clear();
  headings.insert(headings.end(), heading_msgs_.begin(), heading_msgs_.end());
  heading_msgs_.clear();
}

void BynavNmea::GetBynavCorrectedImuData(
    std::vector<bynav_gps_msgs::BynavCorrectedImuDataPtr> &imu_messages) {
  imu_messages.clear();
  imu_messages.insert(imu_messages.end(), corrimudata_msgs_.begin(),
                      corrimudata_msgs_.end());
  corrimudata_msgs_.clear();
}

void BynavNmea::GetGpdopMessages(
    std::vector<bynav_gps_msgs::GpdopPtr> &gpdop_messages) {
  gpdop_messages.clear();
  gpdop_messages.insert(gpdop_messages.end(), gpdop_msgs_.begin(),
                        gpdop_msgs_.end());
  gpdop_msgs_.clear();
}

void BynavNmea::GetGpggaMessages(
    std::vector<bynav_gps_msgs::GpggaPtr> &gpgga_messages) {
  gpgga_messages.clear();
  gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(),
                        gpgga_msgs_.end());
  gpgga_msgs_.clear();
}

void BynavNmea::GetGpgsvMessages(
    std::vector<bynav_gps_msgs::GpgsvPtr> &gpgsv_messages) {
  gpgsv_messages.resize(gpgsv_msgs_.size());
  std::copy(gpgsv_msgs_.begin(), gpgsv_msgs_.end(), gpgsv_messages.begin());
  gpgsv_msgs_.clear();
}

void BynavNmea::GetGphdtMessages(
    std::vector<bynav_gps_msgs::GphdtPtr> &gphdt_messages) {
  gphdt_messages.resize(gphdt_msgs_.size());
  std::copy(gphdt_msgs_.begin(), gphdt_msgs_.end(), gphdt_messages.begin());
  gphdt_msgs_.clear();
}

void BynavNmea::GetGprmcMessages(
    std::vector<bynav_gps_msgs::GprmcPtr> &gprmc_messages) {
  gprmc_messages.clear();
  gprmc_messages.insert(gprmc_messages.end(), gprmc_msgs_.begin(),
                        gprmc_msgs_.end());
  gprmc_msgs_.clear();
}

void BynavNmea::GetInspvaMessages(
    std::vector<bynav_gps_msgs::InspvaPtr> &inspva_messages) {
  inspva_messages.clear();
  inspva_messages.insert(inspva_messages.end(), inspva_msgs_.begin(),
                         inspva_msgs_.end());
  inspva_msgs_.clear();
}

void BynavNmea::GetInspvaxMessages(
    std::vector<bynav_gps_msgs::InspvaxPtr> &inspvax_messages) {
  inspvax_messages.clear();
  inspvax_messages.insert(inspvax_messages.end(), inspvax_msgs_.begin(),
                          inspvax_msgs_.end());
  inspvax_msgs_.clear();
}

void BynavNmea::GetInsstdevMessages(
    std::vector<bynav_gps_msgs::InsstdevPtr> &insstdev_messages) {
  insstdev_messages.clear();
  insstdev_messages.insert(insstdev_messages.end(), insstdev_msgs_.begin(),
                           insstdev_msgs_.end());
  insstdev_msgs_.clear();
}

void BynavNmea::GetTimeMessages(
    std::vector<bynav_gps_msgs::TimePtr> &time_messages) {
  time_messages.resize(time_msgs_.size());
  std::copy(time_msgs_.begin(), time_msgs_.end(), time_messages.begin());
  time_msgs_.clear();
}

void BynavNmea::GetImuMessages(std::vector<sensor_msgs::ImuPtr> &imu_messages) {
  imu_messages.clear();
  imu_messages.insert(imu_messages.end(), imu_msgs_.begin(), imu_msgs_.end());
  imu_msgs_.clear();
}

void BynavNmea::GenerateImuMessages() {
  if (imu_rate_ <= 0.0) {
    ROS_WARN_ONCE("IMU rate has not been configured; cannot produce "
                  "sensor_msgs/Imu messages.");
    return;
  }

  size_t previous_size = imu_msgs_.size();
  while (!corrimudata_queue_.empty() && !inspva_queue_.empty()) {
    bynav_gps_msgs::BynavCorrectedImuDataPtr corrimudata =
        corrimudata_queue_.front();
    bynav_gps_msgs::InspvaPtr inspva = inspva_queue_.front();

    double corrimudata_time =
        corrimudata->gps_week_num * SECONDS_PER_WEEK + corrimudata->gps_seconds;
    double inspva_time =
        inspva->bynav_msg_header.gps_week_num * SECONDS_PER_WEEK +
        inspva->bynav_msg_header.gps_seconds;

    if (std::fabs(corrimudata_time - inspva_time) > IMU_TOLERANCE_S) {
      ROS_DEBUG("INSPVA and CORRIMUDATA were unacceptably far apart.");
      if (corrimudata_time < inspva_time) {
        ROS_DEBUG("Discarding oldest CORRIMUDATA.");
        corrimudata_queue_.pop();
        continue;
      } else {
        ROS_DEBUG("Discarding oldest INSPVA.");
        inspva_queue_.pop();
        continue;
      }
    }
    inspva_queue_.pop();
    corrimudata_queue_.pop();

    sensor_msgs::ImuPtr imu = boost::make_shared<sensor_msgs::Imu>();

    imu->header.stamp = corrimudata->header.stamp;
    imu->orientation = tf::createQuaternionMsgFromRollPitchYaw(
        inspva->roll * DEGREES_TO_RADIANS,
        -(inspva->pitch) * DEGREES_TO_RADIANS,
        -(inspva->azimuth) * DEGREES_TO_RADIANS);

    if (latest_insstdev_) {
      imu->orientation_covariance[0] = std::pow(2, latest_insstdev_->pitch_dev);
      imu->orientation_covariance[4] = std::pow(2, latest_insstdev_->roll_dev);
      imu->orientation_covariance[8] =
          std::pow(2, latest_insstdev_->azimuth_dev);
    } else {
      imu->orientation_covariance[0] = imu->orientation_covariance[4] =
          imu->orientation_covariance[8] = 1e-3;
    }

    imu->angular_velocity.x = corrimudata->pitch_rate * imu_rate_;
    imu->angular_velocity.y = corrimudata->roll_rate * imu_rate_;
    imu->angular_velocity.z = corrimudata->yaw_rate * imu_rate_;
    imu->angular_velocity_covariance[0] = imu->angular_velocity_covariance[4] =
        imu->angular_velocity_covariance[8] = 1e-3;

    imu->linear_acceleration.x = corrimudata->lateral_acceleration * imu_rate_;
    imu->linear_acceleration.y =
        corrimudata->longitudinal_acceleration * imu_rate_;
    imu->linear_acceleration.z = corrimudata->vertical_acceleration * imu_rate_;
    imu->linear_acceleration_covariance[0] =
        imu->linear_acceleration_covariance[4] =
            imu->linear_acceleration_covariance[8] = 1e-3;

    imu_msgs_.push_back(imu);
  }

  size_t new_size = imu_msgs_.size() - previous_size;
  ROS_DEBUG("Created %lu new sensor_msgs/Imu messages.", new_size);
}

void BynavNmea::SetImuRate(double imu_rate, bool imu_rate_forced) {
  ROS_INFO("IMU sample rate: %f", imu_rate);
  imu_rate_ = imu_rate;
  if (imu_rate_forced) {
    imu_rate_forced_ = true;
  }
  enable_imu_ = true;
}

BynavNmea::ReadResult
BynavNmea::ParseBinaryMessage(const BinaryMessage &msg,
                              const ros::Time &stamp) noexcept(false) {
  switch (msg.header_.message_id_) {
  case BestposParser::MESSAGE_ID: {
    bynav_gps_msgs::BynavPositionPtr position =
        bestpos_parser_.ParseBinary(msg);
    position->header.stamp = stamp;
    bynav_positions_.push_back(position);
    bestpos_sync_buffer_.push_back(position);
    break;
  }
  case BynavVelocityParser::MESSAGE_ID: {
    bynav_gps_msgs::BynavVelocityPtr velocity =
        bestvel_parser_.ParseBinary(msg);
    velocity->header.stamp = stamp;
    bynav_velocities_.push_back(velocity);
    bestvel_sync_buffer_.push_back(velocity);
    break;
  }
  case HeadingParser::MESSAGE_ID: {
    bynav_gps_msgs::HeadingPtr heading = heading_parser_.ParseBinary(msg);
    heading->header.stamp = stamp;
    heading_msgs_.push_back(heading);
    break;
  }
  case CorrImuDataParser::MESSAGE_ID: {
    bynav_gps_msgs::BynavCorrectedImuDataPtr imu =
        corrimudata_parser_.ParseBinary(msg);
    imu->header.stamp = stamp;
    corrimudata_msgs_.push_back(imu);
    corrimudata_queue_.push(imu);
    if (corrimudata_queue_.size() > MAX_BUFFER_SIZE) {
      ROS_WARN_THROTTLE(1.0, "CORRIMUDATA queue overflow.");
      corrimudata_queue_.pop();
    }
    GenerateImuMessages();
    break;
  }
  case InspvaParser::MESSAGE_ID: {
    bynav_gps_msgs::InspvaPtr inspva = inspva_parser_.ParseBinary(msg);
    inspva->header.stamp = stamp;
    inspva_msgs_.push_back(inspva);
    inspva_queue_.push(inspva);
    if (inspva_queue_.size() > MAX_BUFFER_SIZE) {
      ROS_WARN_THROTTLE(1.0, "INSPVA queue overflow.");
      inspva_queue_.pop();
    }
    GenerateImuMessages();
    break;
  }
  case InspvaxParser::MESSAGE_ID: {
    bynav_gps_msgs::InspvaxPtr inspvax = inspvax_parser_.ParseBinary(msg);
    inspvax->header.stamp = stamp;
    inspvax_msgs_.push_back(inspvax);
    break;
  }
  case InsstdevParser::MESSAGE_ID: {
    bynav_gps_msgs::InsstdevPtr insstdev = insstdev_parser_.ParseBinary(msg);
    insstdev->header.stamp = stamp;
    insstdev_msgs_.push_back(insstdev);
    latest_insstdev_ = insstdev;
    break;
  }
  default:
    ROS_WARN("Unexpected binary message id: %u", msg.header_.message_id_);
    break;
  }

  return READ_SUCCESS;
}

BynavNmea::ReadResult
BynavNmea::ParseNmeaSentence(const NmeaSentence &sentence,
                             const ros::Time &stamp,
                             double most_recent_utc_time) noexcept(false) {
  if (sentence.id == GpggaParser::MESSAGE_NAME) {
    bynav_gps_msgs::GpggaPtr gpgga = gpgga_parser_.ParseAscii(sentence);

    auto gpgga_time = UtcFloatToSeconds(gpgga->utc_seconds);

    if (most_recent_utc_time < gpgga_time) {
      most_recent_utc_time = gpgga_time;
    }

    gpgga->header.stamp =
        stamp - ros::Duration(most_recent_utc_time - gpgga_time);

    gpgga_msgs_.push_back(std::move(gpgga));
  } else if (sentence.id == GprmcParser::MESSAGE_NAME) {
    bynav_gps_msgs::GprmcPtr gprmc = gprmc_parser_.ParseAscii(sentence);

    auto gprmc_time = UtcFloatToSeconds(gprmc->utc_seconds);

    if (most_recent_utc_time < gprmc_time) {
      most_recent_utc_time = gprmc_time;
    }

    gprmc->header.stamp =
        stamp - ros::Duration(most_recent_utc_time - gprmc_time);

    gprmc_msgs_.push_back(std::move(gprmc));
  } else if (sentence.id == GpgsvParser::MESSAGE_NAME) {
    bynav_gps_msgs::GpgsvPtr gpgsv = gpgsv_parser_.ParseAscii(sentence);
    gpgsv_msgs_.push_back(gpgsv);
  } else if (sentence.id == GphdtParser::MESSAGE_NAME) {
    bynav_gps_msgs::GphdtPtr gphdt = gphdt_parser_.ParseAscii(sentence);
    gphdt_msgs_.push_back(gphdt);
  } else if (sentence.id == PtnlPJKParser::MESSAGE_NAME) {
    bynav_gps_msgs::PtnlPJKPtr position = ptnlpjk_parser_.ParseAscii(sentence);
    bynav_pjk_positions_.push_back(position);
  } else if (sentence.id == GpdopParser::MESSAGE_NAME) {
    auto gpdop = gpdop_parser_.ParseAscii(sentence);
    gpdop_msgs_.push_back(gpdop);
    latest_gpdop_ = gpdop;
  } else {
    ROS_DEBUG_STREAM("Unrecognized NMEA sentence " << sentence.id);
  }

  return READ_SUCCESS;
}

BynavNmea::ReadResult
BynavNmea::ParseBynavSentence(const BynavSentence &sentence,
                              const ros::Time &stamp) noexcept(false) {
  if (sentence.id == "BESTGNSSPOSA") {
    bynav_gps_msgs::BynavPositionPtr position =
        bestpos_parser_.ParseAscii(sentence);
    position->header.stamp = stamp;
    bynav_positions_.push_back(position);
    bestpos_sync_buffer_.push_back(position);
  } else if (sentence.id == "BESTVELA") {
    bynav_gps_msgs::BynavVelocityPtr velocity =
        bestvel_parser_.ParseAscii(sentence);
    velocity->header.stamp = stamp;
    bynav_velocities_.push_back(velocity);
    bestvel_sync_buffer_.push_back(velocity);
  } else if (sentence.id == "HEADINGA") {
    bynav_gps_msgs::HeadingPtr heading = heading_parser_.ParseAscii(sentence);
    heading->header.stamp = stamp;
    heading_msgs_.push_back(heading);
  } else if (sentence.id == "CORRIMUDATAA") {
    bynav_gps_msgs::BynavCorrectedImuDataPtr imu =
        corrimudata_parser_.ParseAscii(sentence);
    imu->header.stamp = stamp;
    corrimudata_msgs_.push_back(imu);
    corrimudata_queue_.push(imu);
    if (corrimudata_queue_.size() > MAX_BUFFER_SIZE) {
      ROS_WARN_THROTTLE(1.0, "CORRIMUDATA queue overflow.");
      corrimudata_queue_.pop();
    }
    GenerateImuMessages();
  } else if (sentence.id == "INSPVAA") {
    bynav_gps_msgs::InspvaPtr inspva = inspva_parser_.ParseAscii(sentence);
    inspva->header.stamp = stamp;
    inspva_msgs_.push_back(inspva);
    inspva_queue_.push(inspva);
    if (inspva_queue_.size() > MAX_BUFFER_SIZE) {
      ROS_WARN_THROTTLE(1.0, "INSPVA queue overflow.");
      inspva_queue_.pop();
    }
    GenerateImuMessages();
  } else if (sentence.id == "INSPVAXA") {
    bynav_gps_msgs::InspvaxPtr inspvax = inspvax_parser_.ParseAscii(sentence);
    inspvax->header.stamp = stamp;
    inspvax_msgs_.push_back(inspvax);
  } else if (sentence.id == "INSSTDEVA") {
    bynav_gps_msgs::InsstdevPtr insstdev =
        insstdev_parser_.ParseAscii(sentence);
    insstdev->header.stamp = stamp;
    insstdev_msgs_.push_back(insstdev);
    latest_insstdev_ = insstdev;
  }
  return READ_SUCCESS;
}

} // namespace bynav_gps_driver
