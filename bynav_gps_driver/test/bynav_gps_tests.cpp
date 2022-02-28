#include <bynav_gps_driver/bynav_nmea.h>

#include <gtest/gtest.h>

#include <ros/package.h>
#include <ros/ros.h>

TEST(BynavGpsTestSuite, testGpsFixParsing) {
  bynav_gps_driver::BynavNmea gps;

  std::string path = ros::package::getPath("bynav_gps_driver");
  gps.wait_for_sync_ = true;

  ASSERT_TRUE(gps.Connect(path + "/test/bestpos-bestvel-gpdop-sync.pcap",
                          bynav_gps_driver::BynavNmea::PCAP));

  std::vector<gps_common::GPSFixPtr> fix_messages;

  while (gps.IsConnected() &&
         gps.ProcessData() == bynav_gps_driver::BynavNmea::READ_SUCCESS) {
    std::vector<gps_common::GPSFixPtr> tmp_messages;
    gps.GetFixMessages(tmp_messages);
    fix_messages.insert(fix_messages.end(), tmp_messages.begin(),
                        tmp_messages.end());
  }

  ASSERT_EQ(22, fix_messages.size());

  EXPECT_DOUBLE_EQ(fix_messages.front()->latitude, 29.443917634921949);
  EXPECT_DOUBLE_EQ(fix_messages.front()->longitude, -98.614755510637181);
  EXPECT_DOUBLE_EQ(fix_messages.front()->speed, 0.041456376659522925);
  EXPECT_DOUBLE_EQ(fix_messages.front()->track, 135.51629763185957);
  EXPECT_DOUBLE_EQ(fix_messages.front()->gdop, 1.9980000257492065);
}

TEST(BynavGpsTestSuite, testCorrImuDataParsing) {
  bynav_gps_driver::BynavNmea gps;

  std::string path = ros::package::getPath("bynav_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/corrimudata.pcap",
                          bynav_gps_driver::BynavNmea::PCAP));

  std::vector<bynav_gps_msgs::BynavCorrectedImuDataPtr> imu_messages;

  while (gps.IsConnected() &&
         gps.ProcessData() == bynav_gps_driver::BynavNmea::READ_SUCCESS) {
    std::vector<bynav_gps_msgs::BynavCorrectedImuDataPtr> tmp_messages;
    gps.GetBynavCorrectedImuData(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(),
                        tmp_messages.end());
  }

  ASSERT_EQ(26, imu_messages.size());

  bynav_gps_msgs::BynavCorrectedImuDataPtr msg = imu_messages.front();
  EXPECT_EQ(1820, msg->gps_week_num);
  EXPECT_DOUBLE_EQ(160205.899999999994, msg->gps_seconds);
  EXPECT_DOUBLE_EQ(0.0000039572689929003956, msg->pitch_rate);
  EXPECT_DOUBLE_EQ(0.0000028926313702935847, msg->roll_rate);
  EXPECT_DOUBLE_EQ(0.0000027924848999730557, msg->yaw_rate);
  EXPECT_DOUBLE_EQ(-0.00062560456243879322, msg->lateral_acceleration);
  EXPECT_DOUBLE_EQ(0.00034037959880710289, msg->longitudinal_acceleration);
  EXPECT_DOUBLE_EQ(-0.0000051257464089797534, msg->vertical_acceleration);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bynav_gps_test_suite",
            ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
