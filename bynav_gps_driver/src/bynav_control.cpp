#include <bynav_gps_driver/bynav_control.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sstream>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

namespace bynav_gps_driver {

BynavControl::BynavControl() {}

bool BynavControl::StartBase(double lat, double lon, float height) {
  std::string str;
  char tempStr[128];

  Write("RTKTYPE BASE");

  sprintf(tempStr, "FIX POSITION %.9lf %.9lf %.3f", lat, lon, height);
  str.assign(tempStr);
  if (!Write(str))
    return false;
  return true;
}

bool BynavControl::StartBase() {
  Write("RTKTYPE BASE");

  if (!Write("FIX AUTO"))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::StopBase() {
  if (!Write("FIX NONE"))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::StartRover() {
  Write("RTKTYPE ROVER");
  return true;
}

bool BynavControl::StopRover() {
  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetPJK(double a, double alpha, double L0, double W0,
                          double FN, double FE, double k0, bool eht) {
  std::string str;
  char tempStr[128];
  sprintf(tempStr, "SET PJKPARA %.3f %.3f %.9lf %.9lf %.9lf %.9lf [%.9lf %s]",
          a, alpha, L0, W0, FN, FE, k0, eht ? "EHT" : "GHT");
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetOBSFreq(int freq) {
  std::string str;
  char tempStr[128];
  sprintf(tempStr, "SET OBSFREQ %.d", freq);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetOutputSource(BYNAV_OUTPUT_SOURCE_TPYE type) {
  std::string str;
  char tempStr[128];
  if (type == OUTPUT_SOURCE_RAW) {
    sprintf(tempStr, "OUTPUTSORCE RAW");
  } else if (type == OUTPUT_SOURCE_KF) {
    sprintf(tempStr, "OUTPUTSORCE KF");
  } else if (type == OUTPUT_SOURCE_INS) {
    sprintf(tempStr, "OUTPUTSORCE INS");
  } else {
    sprintf(tempStr, "OUTPUTSORCE RAW");
  }

  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetShiftDatum(double x, double y, double z) {
  std::string str;
  char tempStr[128];
  sprintf(tempStr, "SET SHIFTDATUM %.9lf %.9lf %.9lf", x, y, z);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::Reboot() {
  Write("REBOOT");
  return true;
}

bool BynavControl::Reset() {
  Write("RESET");
  return true;
}

bool BynavControl::SetSerial(BYNAV_PORT port, int bps) {
  std::string str;
  char tempStr[128];

  if (port < COM1 || port > COM3) {
    return false;
  }

  sprintf(tempStr, "SERIALCONFIG COM%d %d", port, bps);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetINSTranslation(BYNAV_INS_STRANSLATION tranm, double x,
                                     double y, double z, double xsd, double ysd,
                                     double zsd, BYNAV_INPUT_FRAME frame) {
  std::string str;
  char tempStr[128];

  char tranStr[32];
  if (tranm == ANT1) {
    sprintf(tranStr, "ANT1");
  } else if (tranm == ANT2) {
    sprintf(tranStr, "ANT2");
  } else if (tranm == USER) {
    sprintf(tranStr, "USER");
  }

  char frameStr[32];
  if (frame == VEHICLE) {
    sprintf(frameStr, "VEHICLE");
  } else if (frame == IMUBODY) {
    sprintf(frameStr, "IMUBODY");
  }

  sprintf(tempStr, "SETINSTRANSLATION %s %.9f %.9f %.9f %.9f %.9f %.9f %s",
          tranStr, x, y, z, xsd, ysd, zsd, frameStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetINSRotation(BYNAV_INS_ROTATION rotm, double x, double y,
                                  double z, double xsd, double ysd,
                                  double zsd) {
  std::string str;
  char tempStr[128];

  char rotStr[32];
  if (rotm == RBV) {
    sprintf(rotStr, "RBV");
  } else if (rotm == USER1) {
    sprintf(rotStr, "USER");
  }

  sprintf(tempStr, "SETINSROTATION %s %.9f %.9f %.9f %.9f %.9f %.9f", rotStr, x,
          y, z, xsd, ysd, zsd);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetAlignmentVel(double v) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "SETALIGNMENTVEL %.3f", v);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::INSCalibrate(BYNAV_TRIGGER tri) {
  std::string str;
  char tempStr[128];

  char triStr[32];
  if (tri == TRIGGER_NEW) {
    sprintf(triStr, "NEW");
  } else if (tri == TRIGGER_STOP) {
    sprintf(triStr, "STOP");
  } else if (tri == TRIGGER_RESET) {
    sprintf(triStr, "RESET");
  }

  sprintf(tempStr, "INSCALIBRATE RBV %s", triStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetINSProfile(BYNAV_PROFILE tri) {
  std::string str;
  char tempStr[128];

  char proStr[32];
  if (tri == PROFILE_DEFAULT) {
    sprintf(proStr, "DEFAULT");
  } else if (tri == PROFILE_LAND) {
    sprintf(proStr, "LAND");
  }

  sprintf(tempStr, "SETINSPROFILE %s", proStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::AddAuth(std::string auth) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "AUTH ADD %s", auth.data());
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::RemoveAuth() {
  Write("AUTH REMOVE");
  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SaveConfig() {
  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SaveEPHData() {
  Write("SAVEEPHDATA");
  return true;
}

bool BynavControl::RTKTimeout(int s) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "RTKTIMEOUT %d", s);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetECutOff(double angle) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "ECUTOFF %.3f", angle);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetSNRCutOff(double snr) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "SNRCUTOFF %.3f", snr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetBaseLine(double length, double off) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "SETBASELINE ON %.3f %.3f", length, off);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::NetConfig(std::string ip, std::string netmask,
                             std::string gateway) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "IPCONFIG ETHA STATIC %s %s %s", ip.data(), netmask.data(),
          gateway.data());
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::NetConfigDHCP() {
  if (!Write("IPCONFIG ETHA DHCP"))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::ICOMConfig(BYNAV_PORT port, std::string host, int net_port,
                              BYNAV_NET_PROTO_TPYE proto) {
  std::string str;
  char tempStr[128];

  if (port < ICOM1 || port > ICOM4) {
    return false;
  }

  sprintf(tempStr, "ICOMCONFIG ICOM%d %s %s:%d", (port - 10),
          (proto == NP_TCP) ? "TCP" : "UDP", host.data(), net_port);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::ICOMDisable(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  if (port < ICOM1 || port > ICOM4) {
    return false;
  }

  sprintf(tempStr, "ICOMCONFIG ICOM%d DISABLED", (port - 10));
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetFrequencyOut(int plusewidth, int period,
                                   BYNAV_TTL_TPYE ttl,
                                   BYNAV_EVENT_OUT_TPYE instance) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "FREQUENCYOUT ENABLE %d %d %s %d",
          (ttl == TTL_POSITIVE) ? "POSITIVE" : "NEGATIVE", instance);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::Freset(char *cmd) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "FRESET %s", cmd);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::DualAntennaPower(bool flag) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "DUALANTENNAPOWER %s", flag ? "ON" : "OFF");
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::SetGPSRefWeek(int weeknum) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "GPSREFWEEK %d", weeknum);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::LogRTCM3(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];
  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  sprintf(tempStr, "UNLOGALL %s", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "INTERFACEMODE %s BYNAV RTCM ON", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1074 ONTIME 1", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1084 ONTIME 1", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1094 ONTIME 1", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1114 ONTIME 1", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1124 ONTIME 1", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1006 ONTIME 5", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s RTCM1033 ONTIME 10", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::LogRAWEPH(BYNAV_PORT port, int interval, int raw_eph) {
  std::string str;
  char tempStr[128], intertemp[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  if (raw_eph > 0)
    sprintf(intertemp, "ONTIME %g", (float)raw_eph);
  else if (raw_eph < 0)
    strcpy(intertemp, "ONCHANGED");

  if (raw_eph != 0) {
    sprintf(tempStr, "LOG %s RAWEPHEMB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s IONUTCB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s GLOEPHEMERISB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s BDSEPHEMERISB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s BDSIONOB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s BDSALMANACB %s", portStr, intertemp);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s VERSION ONCE", portStr);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  return true;
}

bool BynavControl::LogRAW(BYNAV_PORT port, int interval, int raw_eph) {
  std::string str;
  char tempStr[128], intertemp[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }
  UnlogAll(port);
  sprintf(tempStr, "INTERFACEMODE %s BYNAV BYNAV on", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;
  if (interval != 0) {
    sprintf(tempStr, "LOG %s RANGECMPB ONTIME %d", portStr, interval);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s TIMEB ONTIME %d", portStr, interval);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s BESTPOSB ONTIME %d", portStr, interval);
    str.assign(tempStr);
    if (!Write(str))
      return false;

    sprintf(tempStr, "LOG %s BESTGNSSVELB ONTIME %d", portStr, interval);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  LogRAWEPH(port, interval, raw_eph);

  Write("SAVECONFIG");

  return true;
}

bool BynavControl::LogRecord(BYNAV_PORT port, int interval) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }
  UnlogAll(port);

  sprintf(tempStr, "LOG %s RANGECMPB ONTIME %d", portStr, interval);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s GPSEPHEMB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s BDSEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s GLOEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s IONUTCB %s", portStr, interval);
  str.assign(tempStr);
  if (!Write(str))
    false;

  sprintf(tempStr, "LOG %s BDSALMANACB %s", portStr, interval);
  str.assign(tempStr);
  if (!Write(str))
    false;

  Write("SAVECONFIG");

  return true;
}

bool BynavControl::StopRecord(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  sprintf(tempStr, "LOG %s RAWEPHEMB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s BDSEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s GLOEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;
  return true;
}

bool BynavControl::StopDiff(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  sprintf(tempStr, "UNLOGALL %s", portStr);
  str.assign(tempStr);
  if (!Write(str)) {
    usleep(500 * 1000);
    if (!Write(str)) {
      usleep(500 * 1000);
      if (!Write(str)) {
        usleep(500 * 1000);
        if (!Write(str)) {
          return false;
        }
      }
    }
  }

  sprintf(tempStr, "INTERFACEMODE %s BYNAV BYNAV on", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  return true;
}

bool BynavControl::LogDiffData(BYNAV_PORT port, double lat, double lon,
                               float height) {
  bool res = true;
  UnlogAll(port);
  StartBase(lat, lon, height);
  return LogRTCM3(port);
}

bool BynavControl::RTKMode(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  sprintf(tempStr, "INTERFACEMODE %s AUTO BYNAV", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "UNLOGALL %s", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;
  return true;
}

bool BynavControl::InrerfaceMode(BYNAV_PORT port, BYNAV_PROTO in,
                                 BYNAV_PROTO out) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= COM1 && port <= COM3) {
    sprintf(portStr, "COM%d", port);
  } else {
    return false;
  }

  char inStr[32];
  if (in == PROTO_NONE) {
    sprintf(inStr, "NONE");
  } else if (in == PROTO_AUTO) {
    sprintf(inStr, "AUTO");
  } else if (in == PROTO_BYNAV) {
    sprintf(inStr, "BYNAV");
  } else if (in == PROTO_RTCM) {
    sprintf(inStr, "RTCM");
  } else if (in == PROTO_FPGA) {
    sprintf(inStr, "FPGA");
  }
  char outStr[32];
  if (in == PROTO_NONE) {
    sprintf(outStr, "NONE");
  } else if (in == PROTO_AUTO) {
    sprintf(outStr, "AUTO");
  } else if (in == PROTO_BYNAV) {
    sprintf(outStr, "BYNAV");
  } else if (in == PROTO_RTCM) {
    sprintf(outStr, "RTCM");
  } else if (in == PROTO_FPGA) {
    sprintf(outStr, "FPGA");
  }

  sprintf(tempStr, "INTERFACEMODE %s %s %s", portStr, inStr, outStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;
}

bool BynavControl::UnlogAll(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  char portStr[32];
  if (port >= ICOM1) {
    sprintf(portStr, "ICOM%d", (port - 10));
  } else {
    sprintf(portStr, "COM%d", port);
  }

  sprintf(tempStr, "UNLOGALL %s", portStr);
  str.assign(tempStr);
  if (!Write(str)) {
    usleep(500 * 1000);
    if (!Write(str)) {
      usleep(500 * 1000);
      if (!Write(str)) {
        usleep(500 * 1000);
        if (!Write(str)) {
          return false;
        }
      }
    }
  }

  sprintf(tempStr, "INTERFACEMODE %s BYNAV BYNAV on", portStr);
  str.assign(tempStr);
  if (!Write(str))
    ;

  return true;
}

bool BynavControl::UnlogAllPorts() {
  if (!Write("UNLOGALL"))
    return false;
  return true;
}

bool BynavControl::Init() {
  if (!Write("LOG VERSION")) {
    sleep(1);
    if (!Write("LOG VERSION")) {
      return false;
    }
  }

  Write("UNLOGALL");
  if (!Write("LOG BESTPOSB ONTIME 1"))
    return false;
  if (!Write("LOG RTKDOPB ONCHANGED"))
    return false;
  if (!Write("LOG TIMEB ONTIME 1"))
    return false;

  if (!Write("IPCONFIG DHCP"))
    ;

  if (!Write("NMEATALKER AUTO"))
    ;

  if (!Write("NETPORTCONFIG ICOM1 TCP 3001"))
    ;
  if (!Write("NETPORTCONFIG ICOM2 TCP 3002"))
    ;
  if (!Write("NETPORTCONFIG ICOM3 TCP 3003"))
    ;
  if (!Write("NETPORTCONFIG ICOM4 TCP 3004"))
    ;

  Write("INTERFACEMODE ICOM1 BYNAV BYNAV ON");
  Write("INTERFACEMODE ICOM2 BYNAV BYNAV ON");
  Write("INTERFACEMODE ICOM3 BYNAV BYNAV ON");
  Write("INTERFACEMODE ICOM4 BYNAV BYNAV ON");

  if (!Write("LOG RANGEB ONTIME 1"))
    ;
  if (!Write("LOG SATVIS2B ONTIME 1"))
    ;
  if (!Write("LOG RTKDOPB ONTIME 1"))
    ;

  Write("FIX NONE");

  usleep(200 * 1000);
  Write("INTERFACEMODE COM1 BYNAV BYNAV ON");
  Write("INTERFACEMODE COM2 BYNAV BYNAV ON");
  Write("INTERFACEMODE COM3 BYNAV BYNAV ON");

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::NtripServer(BYNAV_PORT port, BYNAV_NTRIP_VER proto,
                               std::string username, std::string password,
                               std::string endpoint, int netport) {
  std::string str;
  char tempStr[128];

  if (port < NCOM1 || port > NCOM2)
    return false;

  char netPortStr[10];
  if (netport == -1) {
    sprintf(netPortStr, "ALL");
  } else {
    sprintf(netPortStr, "%d", netport);
  }

  sprintf(tempStr, "NTRIPCONFIG NCOM%d SERVER V%d %s NTRIP %s %s %s",
          (port - 20), proto, endpoint.data(), username.data(), password.data(),
          netPortStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::NtripClient(BYNAV_PORT port, BYNAV_NTRIP_VER proto,
                               std::string username, std::string password,
                               std::string endpoint, int netport) {
  std::string str;
  char tempStr[128];

  if (port < NCOM1 || port > NCOM2)
    return false;

  char netPortStr[10];
  if (netport == -1) {
    sprintf(netPortStr, "ALL");
  } else {
    sprintf(netPortStr, "%d", netport);
  }

  sprintf(tempStr, "NTRIPCONFIG NCOM%d CLIENT V%d %s NTRIP %s %s %s",
          (port - 20), proto, endpoint.data(), username.data(), password.data(),
          netPortStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::NtripDisable(BYNAV_PORT port) {
  std::string str;
  char tempStr[128];

  if (port < NCOM1 || port > NCOM2)
    return false;

  sprintf(tempStr, "NTRIPCONFIG NCOM%d DISABLED", (port - 20));
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}

bool BynavControl::QualityCheckEnable() {
  if (!Write("QUALITYCHECK POS ON"))
    return false;
  Write("SAVECONFIG");
  return true;
}

bool BynavControl::QualityCheckDisable() {
  if (!Write("QUALITYCHECK POS OFF"))
    return false;
  Write("SAVECONFIG");
  return true;
}

bool BynavControl::DNSConfig(std::string dns) {
  std::string str;
  char tempStr[128];

  sprintf(tempStr, "DNSCONFIG 1 %s", dns.data());
  str.assign(tempStr);
  if (!Write(str))
    return false;

  Write("SAVECONFIG");
  return true;
}
} // namespace bynav_gps_driver
