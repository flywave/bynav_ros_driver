#include <boost/make_shared.hpp>
#include <bynav_gps_driver/parsers/gpgsa.h>

const std::string bynav_gps_driver::GpgsaParser::MESSAGE_NAME = "GPGSA";

uint32_t bynav_gps_driver::GpgsaParser::GetMessageId() const { return 0; }

const std::string bynav_gps_driver::GpgsaParser::GetMessageName() const {
  return MESSAGE_NAME;
}

bynav_gps_msgs::GpgsaPtr bynav_gps_driver::GpgsaParser::ParseAscii(
    const bynav_gps_driver::NmeaSentence &sentence) noexcept(false) {
  const size_t LENGTH = 18;
  if (sentence.body.size() != LENGTH) {
    std::stringstream error;
    error << "Expected GPGSA length " << LENGTH << ", actual length "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bynav_gps_msgs::GpgsaPtr msg = boost::make_shared<bynav_gps_msgs::Gpgsa>();
  msg->message_id = sentence.body[0];
  msg->auto_manual_mode = sentence.body[1];
  ParseUInt8(sentence.body[2], msg->fix_mode);

  msg->sv_ids.resize(12, 0);
  size_t n_svs = 0;
  for (std::vector<std::string>::const_iterator id = sentence.body.begin() + 3;
       id < sentence.body.begin() + 15; ++id) {
    if (!id->empty()) {
      ParseUInt8(*id, msg->sv_ids[n_svs]);
      ++n_svs;
    }
  }
  msg->sv_ids.resize(n_svs);

  ParseFloat(sentence.body[15], msg->pdop);
  ParseFloat(sentence.body[16], msg->hdop);
  ParseFloat(sentence.body[17], msg->vdop);
  return msg;
}
