#ifndef BYNAV_CONTROL_H_
#define BYNAV_CONTROL_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

#include <bynav_gps_driver/bynav_connection.h>
#include <bynav_gps_msgs/BynavConfig.h>

namespace bynav_gps_driver {

class BynavControl : public BynavConnection {
public:
  enum BYNAV_OUTPUT_SOURCE_TPYE {
    OUTPUT_SOURCE_RAW = 0,
    OUTPUT_SOURCE_KF,
    OUTPUT_SOURCE_INS,
    OUTPUT_SOURCE_NONE
  };

  enum BYNAV_TTL_TPYE { TTL_POSITIVE = 0, TTL_NEGATIVE = 1 };

  enum BYNAV_EVENT_OUT_TPYE { EVENT_OUT1 = 0, PPS = 1 };

  enum BYNAV_NET_PROTO_TPYE { NP_TCP = 0, NP_UDP = 1 };

  enum BYNAV_PORT {
    ALL = 0,
    COM1 = 1,
    COM2 = 2,
    COM3 = 3,
    ICOM1 = 11,
    ICOM2 = 12,
    ICOM3 = 13,
    ICOM4 = 14,
    NCOM1 = 21,
    NCOM2 = 22
  };

  enum BYNAV_NTRIP_VER {
    NTRIP_V1 = 1,
    NTRIP_V2 = 2,
  };

  enum BYNAV_PROTO {
    PROTO_NONE = 0,
    PROTO_AUTO = 1,
    PROTO_BYNAV = 2,
    PROTO_RTCM = 3,
    PROTO_FPGA = 4
  };

  enum BYNAV_INS_STRANSLATION { ANT1 = 0, ANT2 = 1, USER = 2 };

  enum BYNAV_INS_ROTATION { USER1 = 0, RBV = 1 };

  enum BYNAV_INPUT_FRAME {
    VEHICLE = 0,
    IMUBODY = 1,
  };

  enum BYNAV_TRIGGER {
    TRIGGER_NEW = 0,
    TRIGGER_STOP = 1,
    TRIGGER_RESET = 2,
  };

  enum BYNAV_PROFILE { PROFILE_DEFAULT = 0, PROFILE_LAND = 1 };

  BynavControl();
  virtual ~BynavControl() = default;

  bool SetupConfig(const bynav_gps_msgs::BynavConfig &conf);

protected:
  bool UnlogAll(BYNAV_PORT port);
  bool UnlogAllPorts();
  bool RTKMode(BYNAV_PORT port);
  bool InrerfaceMode(BYNAV_PORT port, BYNAV_PROTO in, BYNAV_PROTO out);

  bool StartBase(double lat, double lon, float height);
  bool StartBase();
  bool StopBase();
  bool StartRover();
  bool StopRover();
  bool SaveConfig();
  bool SaveEPHData();

  bool Reboot();
  bool Reset();
  bool SetSerial(BYNAV_PORT port, int bps);
  bool RTKTimeout(int s);

  bool SetINSTranslation(BYNAV_INS_STRANSLATION tranm, double x, double y,
                         double z, double xsd, double ysd, double zsd,
                         BYNAV_INPUT_FRAME frame);
  bool SetINSRotation(BYNAV_INS_ROTATION rotm, double x, double y, double z,
                      double xsd, double ysd, double zsd);
  bool SetAlignmentVel(double v);
  bool INSCalibrate(BYNAV_TRIGGER tri);
  bool SetINSProfile(BYNAV_PROFILE pr);

  bool AddAuth(std::string auth);
  bool RemoveAuth();

  bool NetConfig(std::string ip, std::string netmask, std::string gateway);
  bool NetConfigDHCP();

  bool ICOMConfig(BYNAV_PORT port, std::string host, int net_port,
                  BYNAV_NET_PROTO_TPYE proto = NP_TCP);
  bool ICOMDisable(BYNAV_PORT port);

  bool SetPJK(double a, double alpha, double L0, double W0, double FN,
              double FE, double k0 = 1.0, bool eht = true);
  bool SetShiftDatum(double x, double y, double z);
  bool SetOBSFreq(int freq);

  bool SetFrequencyOut(int plusewidth, int period, BYNAV_TTL_TPYE ttl,
                       BYNAV_EVENT_OUT_TPYE instance = PPS);

  bool Freset(char *cmd);
  bool DualAntennaPower(bool flag);
  bool SetGPSRefWeek(int weeknum);
  bool SetECutOff(double angle);
  bool SetBaseLine(double length, double off);
  bool SetSNRCutOff(double snr);
  bool SetOutputSource(BYNAV_OUTPUT_SOURCE_TPYE type);

  bool NtripServer(BYNAV_PORT port, BYNAV_NTRIP_VER proto, std::string username,
                   std::string password, std::string endpoint = "NTRIP",
                   int netport = -1);
  bool NtripClient(BYNAV_PORT port, BYNAV_NTRIP_VER proto, std::string username,
                   std::string password, std::string endpoint = "NTRIP",
                   int netport = -1);
  bool NtripDisable(BYNAV_PORT port);
  bool DNSConfig(std::string dns);

  bool QualityCheckEnable();
  bool QualityCheckDisable();

  bool LogRTCM3(BYNAV_PORT port);
  bool LogDiffData(BYNAV_PORT port, double lat, double lon, float height);
  bool LogRecord(BYNAV_PORT port, int interval);
  bool StopRecord(BYNAV_PORT port);
  bool StopDiff(BYNAV_PORT port);
  bool Init();
};

} // namespace bynav_gps_driver
#endif // BYNAV_CONTROL_H_
