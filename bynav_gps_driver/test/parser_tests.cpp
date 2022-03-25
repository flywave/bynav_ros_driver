#include <bynav_gps_driver/bynav_message_extractor.h>
#include <bynav_gps_driver/parsers/bestpos.h>
#include <bynav_gps_driver/parsers/gpgga.h>
#include <bynav_gps_driver/parsers/gpgsv.h>
#include <bynav_gps_driver/parsers/gphdt.h>
#include <bynav_gps_driver/parsers/heading.h>
#include <bynav_gps_driver/parsers/ptnlpjk.h>

#include <bynav_gps_driver/parsers/corrimudata.h>
#include <bynav_gps_driver/parsers/inscov.h>
#include <bynav_gps_driver/parsers/inspva.h>
#include <bynav_gps_driver/parsers/insstdev.h>
#include <gtest/gtest.h>

TEST(ParserTestSuite, testBestposAsciiParsing) {
  bynav_gps_driver::BestposParser parser;
  std::string bestpos_str =
      "#BESTPOSA,ICOM1,0,87.5,FINESTEERING,1956,157432.000,00000800,7145,6938;"
      "SOL_COMPUTED,SINGLE,29.44391220792,-98.61476921244,261.4344,-26.0000,"
      "WGS84,2.1382,"
      "3.1092,4.0429,\"\",0.000,0.000,8,8,8,8,0,06,00,03*ecf2202b\r\n"
      "#BESTPOSA,COM1,0,83.5,FINESTEERING,1419,336148.000,02000040,6145,2724;"
      "SOL_COMPUTED,SINGLE,"
      "51.11636418888,-114.03832502118,1064.9520,-16.2712,WGS84,1.6961,1.3636,"
      "3.6449,\"\","
      "0.000,0.000,8,8,8,8,0,0,0,06,0,03*f181ad10\r\n"
      "#BESTPOSA,COM1,0,78.5,FINESTEERING,1419,336208.000,02000040,6145,2724;"
      "SOL_COMPUTED,"
      "NARROW_INT,51.11635910984,-114.03833105168,1063.8416,-16.2712,WGS84,0."
      "0135,0.0084,"
      "0.0172,\"AAAA\",1.000,0.000,8,8,8,8,0,01,0,03*072421c0\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(bestpos_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(3, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::BynavPositionPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ(0x800, msg->bynav_msg_header.receiver_status.original_status_code);

  ASSERT_EQ(true, msg->bynav_msg_header.receiver_status.usb_buffer_overrun);

  ASSERT_EQ("SOL_COMPUTED", msg->solution_status);
  ASSERT_DOUBLE_EQ(29.44391220792, msg->lat);
  ASSERT_DOUBLE_EQ(-98.61476921244, msg->lon);
}

TEST(ParserTestSuite, testGpggaParsing) {
  bynav_gps_driver::GpggaParser parser;
  std::string sentence_str =
      "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,"
      "1048.47,M,-16.27,M,08,AAAA*60\r\n"
      "$GPGGA,134658.00,5106.9792,N,11402.3003,W,0,09,1.0,"
      "1048.47,M,-16.27,M,08,AAAA*62\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(2, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(0, bynav_sentences.size());

  bynav_gps_driver::NmeaSentence sentence = nmea_sentences.front();

  ASSERT_EQ(parser.GetMessageName(), sentence.id);

  bynav_gps_msgs::GpggaPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);
  ASSERT_EQ(bynav_gps_msgs::Gpgga::GPS_QUAL_PSEUDORANGE_DIFFERENTIAL,
            msg->gps_qual);
  ASSERT_DOUBLE_EQ(51.116319999999995, msg->lat);
  ASSERT_STREQ("N", msg->lat_dir.c_str());
  ASSERT_DOUBLE_EQ(114.03833833333334, msg->lon);
  ASSERT_STREQ("W", msg->lon_dir.c_str());
  ASSERT_EQ(9, msg->num_sats);
  ASSERT_DOUBLE_EQ(1.0, msg->hdop);
  ASSERT_DOUBLE_EQ(1048.469970703125, msg->alt);
  ASSERT_STREQ("M", msg->altitude_units.c_str());
  ASSERT_DOUBLE_EQ(-16.270000457763672, msg->undulation);
  ASSERT_STREQ("M", msg->undulation_units.c_str());
  ASSERT_EQ(8, msg->diff_age);
  ASSERT_STREQ("AAAA", msg->station_id.c_str());

  sentence = nmea_sentences.at(1);
  msg = parser.ParseAscii(sentence);

  ASSERT_EQ(bynav_gps_msgs::Gpgga::GPS_QUAL_INVALID, msg->gps_qual);
}

TEST(ParserTestSuite, testCorrimudataAsciiParsing) {
  bynav_gps_driver::CorrImuDataParser parser;
  std::string sentence_str =
      "#CORRIMUDATAA,COM1,0,77.5,FINESTEERING,1769,237601.000,02000020,"
      "bdba,12597;1769,237601.000000000,-0.000003356,0.000002872,0.000001398,0."
      "000151593,"
      "0.000038348,-0.000078820*e370e1d9\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(1, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::BynavCorrectedImuDataPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);
  ASSERT_EQ(1769, msg->gps_week_num);
  ASSERT_DOUBLE_EQ(237601.0, msg->gps_seconds);
  ASSERT_DOUBLE_EQ(-0.000003356, msg->pitch_rate);
  ASSERT_DOUBLE_EQ(0.000002872, msg->roll_rate);
  ASSERT_DOUBLE_EQ(0.000001398, msg->yaw_rate);
  ASSERT_DOUBLE_EQ(0.000151593, msg->lateral_acceleration);
  ASSERT_DOUBLE_EQ(0.000038348, msg->longitudinal_acceleration);
  ASSERT_DOUBLE_EQ(-0.00007882, msg->vertical_acceleration);
}

TEST(ParserTestSuite, testGpgsvParsing) {
  bynav_gps_driver::GpgsvParser parser;
  std::string sentence_str =
      "$GPGSV,3,3,11,12,07,00.,32,13,03,227,36,22,0.,041,*4A\r\n"
      "$GPGSV,1,1,00,,,,*79\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(2, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(0, bynav_sentences.size());

  bynav_gps_driver::NmeaSentence sentence = nmea_sentences.front();

  ASSERT_EQ(parser.GetMessageName(), sentence.id);
  ASSERT_FALSE(sentence.body.empty());

  bynav_gps_msgs::GpgsvPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ(3, msg->n_msgs);
  ASSERT_EQ(3, msg->msg_number);
  ASSERT_EQ(3, msg->satellites.size());
  ASSERT_EQ(11, msg->n_satellites);
  ASSERT_EQ(12, msg->satellites[0].prn);
  ASSERT_EQ(7, msg->satellites[0].elevation);
  ASSERT_EQ(0, msg->satellites[0].azimuth);
  ASSERT_EQ(32, msg->satellites[0].snr);
  ASSERT_EQ(13, msg->satellites[1].prn);
  ASSERT_EQ(3, msg->satellites[1].elevation);
  ASSERT_EQ(227, msg->satellites[1].azimuth);
  ASSERT_EQ(36, msg->satellites[1].snr);
  ASSERT_EQ(22, msg->satellites[2].prn);
  ASSERT_EQ(0, msg->satellites[2].elevation);
  ASSERT_EQ(41, msg->satellites[2].azimuth);
  ASSERT_EQ(-1, msg->satellites[2].snr);

  msg = parser.ParseAscii(nmea_sentences.at(1));

  ASSERT_NE(msg.get(), nullptr);
  ASSERT_EQ(0, msg->satellites.size());
}

TEST(ParserTestSuite, testGphdtParsing) {
  bynav_gps_driver::GphdtParser parser;
  std::string sentence_str = "$GPHDT,275.432,T*30\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(1, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(0, bynav_sentences.size());

  bynav_gps_driver::NmeaSentence sentence = nmea_sentences.front();

  ASSERT_EQ(parser.GetMessageName(), sentence.id);
  ASSERT_FALSE(sentence.body.empty());

  bynav_gps_msgs::GphdtPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_DOUBLE_EQ(275.432, msg->heading);
  ASSERT_EQ("T", msg->t);
}

TEST(ParserTestSuite, testInspvaAsciiParsing) {
  bynav_gps_driver::InspvaParser parser;
  std::string sentence_str =
      "#INSPVAA,COM1,0,31.0,FINESTEERING,1264,144088.000,02040000,"
      "5615,1541;1264,144088.002284950,51.116827527,-114.037738908,401."
      "191547167,354.846489850,"
      "108.429407241,-10.837482850,1.116219952,-3.476059035,7.372686190,INS_"
      "ALIGNMENT_COMPLETE*a2913d36\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(1, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::InspvaPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ(1264, msg->week);
  ASSERT_DOUBLE_EQ(144088.002284950, msg->seconds);
  ASSERT_DOUBLE_EQ(51.116827527, msg->latitude);
  ASSERT_DOUBLE_EQ(-114.037738908, msg->longitude);
  ASSERT_DOUBLE_EQ(401.191547167, msg->height);
  ASSERT_DOUBLE_EQ(354.846489850, msg->north_velocity);
  ASSERT_DOUBLE_EQ(108.429407241, msg->east_velocity);
  ASSERT_DOUBLE_EQ(-10.837482850, msg->up_velocity);
  ASSERT_DOUBLE_EQ(1.116219952, msg->roll);
  ASSERT_DOUBLE_EQ(-3.476059035, msg->pitch);
  ASSERT_DOUBLE_EQ(7.372686190, msg->azimuth);
  ASSERT_EQ("INS_ALIGNMENT_COMPLETE", msg->status);
}

TEST(ParserTestSuite, testInsstdevAsciiParsing) {
  bynav_gps_driver::InsstdevParser parser;
  std::string sentence_str =
      "#INSSTDEVA,COM1,0,78.0,FINESTEERING,1907,233990.000,02000020,"
      "3e6d,32768;0.4372,0.3139,0.7547,0.0015,0.0015,0.0014,3.7503,3.7534,5."
      "1857,26000005,"
      "0,0,01ffd1bf,0*3deca7d2\r\n";
  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(1, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::InsstdevPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_FLOAT_EQ(0.4372, msg->latitude_dev);
  ASSERT_FLOAT_EQ(0.3139, msg->longitude_dev);
  ASSERT_FLOAT_EQ(0.7547, msg->height_dev);
  ASSERT_FLOAT_EQ(0.0015, msg->north_velocity_dev);
  ASSERT_FLOAT_EQ(0.0015, msg->east_velocity_dev);
  ASSERT_FLOAT_EQ(0.0014, msg->up_velocity_dev);
  ASSERT_FLOAT_EQ(3.7503, msg->roll_dev);
  ASSERT_FLOAT_EQ(3.7534, msg->pitch_dev);
  ASSERT_FLOAT_EQ(5.1857, msg->azimuth_dev);
  ASSERT_EQ(26000005, msg->extended_solution_status.original_mask);
}

TEST(ParserTestSuite, testBestxyzAsciiParsing) {
  bynav_gps_driver::PtnlPJKParser parser;
  std::string ptnlpjk_str =
      "#BESTXYZA,COM1,0,55.0,FINESTEERING,1419,340033.000,02000040,d821,2724;"
      "SOL_COMPUTED,NARROW_INT,-1634531.5683,-3664618.0326,4942496.3270,0.0099,"
      "0.0219,0.0115,SOL_COMPUTED,NARROW_INT,0.0011,-0.0049,-0.0001,0.0199,0."
      "0439,"
      "0.0230,\"AAAA\",0.250,1.000,0.000,12,11,11,11,0,01,0,33*1fe2f509\r\n";

  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(ptnlpjk_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(1, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::PtnlPJKPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ("SOL_COMPUTED", msg->solution_status);
  ASSERT_EQ("NARROW_INT", msg->position_type);
  ASSERT_DOUBLE_EQ(-1634531.5683, msg->x);
  ASSERT_DOUBLE_EQ(-3664618.0326, msg->y);
  ASSERT_DOUBLE_EQ(4942496.3270, msg->z);
  ASSERT_FLOAT_EQ(0.0099, msg->x_sigma);
  ASSERT_FLOAT_EQ(0.0219, msg->y_sigma);
  ASSERT_FLOAT_EQ(0.0115, msg->z_sigma);
  ASSERT_EQ("SOL_COMPUTED", msg->velocity_solution_status);
  ASSERT_EQ("NARROW_INT", msg->velocity_type);
  ASSERT_DOUBLE_EQ(0.0011, msg->x_vel);
  ASSERT_DOUBLE_EQ(-0.0049, msg->y_vel);
  ASSERT_DOUBLE_EQ(-0.0001, msg->z_vel);
  ASSERT_FLOAT_EQ(0.0199, msg->x_vel_sigma);
  ASSERT_FLOAT_EQ(0.0439, msg->y_vel_sigma);
  ASSERT_FLOAT_EQ(0.0230, msg->z_vel_sigma);
  ASSERT_EQ("\"AAAA\"", msg->base_station_id);
  ASSERT_FLOAT_EQ(0.250, msg->velocity_latency);
  ASSERT_FLOAT_EQ(1.000, msg->diff_age);
  ASSERT_FLOAT_EQ(0.000, msg->solution_age);
  ASSERT_EQ(12, msg->num_satellites_tracked);
  ASSERT_EQ(11, msg->num_satellites_used_in_solution);
  ASSERT_EQ(11, msg->num_gps_and_glonass_l1_used_in_solution);
  ASSERT_EQ(11, msg->num_gps_and_glonass_l1_and_l2_used_in_solution);
  ASSERT_EQ(1, msg->extended_solution_status.original_mask);
}

TEST(ParserTestSuite, testHeadingAsciiParsing) {
  bynav_gps_driver::HeadingParser parser;
  std::string heading_str =
      "#HEADINGA,COM1,0,39.5,FINESTEERING,1622,422892.200,02040000,f9bf,6521;"
      "SOL_COMPUTED,NARROW_INT,0.927607417,178.347869873,-1.3037414550,0,0."
      "261901051,0.391376048,"
      "\"R222\",\"AAAA\",18,17,17,16,0,01,0,33*8c48d77c\r\n"
      "#HEADINGA,COM1,0,39.5,FINESTEERING,1622,422892.200,02040000,f9bf,6521;"
      "SOL_COMPUTED,NARROW_INT,0.927607417,178.347869873,-1.3037414550,0,0."
      "261901051,0.391376048,"
      "\"R222\",\"AAAA\",18,17,17,16,4,01,0,33*d1a48670\r\n";

  std::string extracted_str;

  bynav_gps_driver::BynavMessageExtractor extractor;

  std::vector<bynav_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<bynav_gps_driver::BynavSentence> bynav_sentences;
  std::vector<bynav_gps_driver::BinaryMessage> binary_messages;
  std::vector<bynav_gps_driver::BinaryMicroMessage> binary_mirco_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(heading_str, nmea_sentences,
                                    bynav_sentences, binary_messages,
                                    binary_mirco_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(2, bynav_sentences.size());

  bynav_gps_driver::BynavSentence sentence = bynav_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  bynav_gps_msgs::HeadingPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ("SOL_COMPUTED", msg->solution_status);
  ASSERT_EQ("NARROW_INT", msg->position_type);
  ASSERT_FLOAT_EQ(0.927607417, msg->baseline_length);
  ASSERT_FLOAT_EQ(178.347869873, msg->heading);
  ASSERT_FLOAT_EQ(-1.3037414550, msg->pitch);
  ASSERT_FLOAT_EQ(0.261901051, msg->heading_sigma);
  ASSERT_FLOAT_EQ(0.391376048, msg->pitch_sigma);
  ASSERT_EQ("\"R222\"", msg->rover_station_id);
  ASSERT_EQ("\"AAAA\"", msg->master_station_id);
  ASSERT_EQ(18, msg->num_satellites_tracked);
  ASSERT_EQ(17, msg->num_satellites_used_in_solution);
  ASSERT_EQ(17, msg->num_satellites_above_elevation_mask_angle);
  ASSERT_EQ(16, msg->num_satellites_above_elevation_mask_angle_l2);
  ASSERT_EQ(bynav_gps_msgs::Heading::SOURCE_PRIMARY_ANTENNA,
            msg->solution_source);
  ASSERT_EQ(1, msg->extended_solution_status.original_mask);

  msg = parser.ParseAscii(bynav_sentences.at(1));

  ASSERT_NE(msg.get(), nullptr);
  ASSERT_EQ(bynav_gps_msgs::Heading::SOURCE_SECONDARY_ANTENNA,
            msg->solution_source);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
