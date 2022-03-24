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
  char tempStr[256];

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

  sprintf(tempStr, "FREQUENCYOUT ENABLE %d %d %s %d", plusewidth, period,
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

  sprintf(tempStr, "LOG %s GALEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s QZSSEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

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

  sprintf(tempStr, "LOG %s GALEPHEMERISB ONCHANGED", portStr);
  str.assign(tempStr);
  if (!Write(str))
    return false;

  sprintf(tempStr, "LOG %s QZSSEPHEMERISB ONCHANGED", portStr);
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
  return true;
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

bool BynavControl::SetupConfig(const bynav_gps_msgs::BynavConfig &conf) {
  if (!conf.modes.empty()) {
    for (auto &mode : conf.modes) {
      std::string str;
      char tempStr[128];

      sprintf(tempStr, "INTERFACEMODE %s %s %s ON", mode.port.data(),
              mode.formatin.data(), mode.formatout.data());
      str.assign(tempStr);
      if (!Write(str))
        return false;
    }
  }

  if (!conf.serials.empty()) {
    for (auto &serial : conf.serials) {
      std::string str;
      char tempStr[128];

      sprintf(tempStr, "SERIALCONFIG COM%d %d", serial.port, serial.bps);
      str.assign(tempStr);
      if (!Write(str))
        return false;
    }
  }

  if (!conf.icom_configs.empty()) {
    for (auto &icom : conf.icom_configs) {
      std::string str;
      char tempStr[128];

      sprintf(tempStr, "ICOMCONFIG ICOM%d %s %d", icom.port,
              icom.protocol.data(), icom.endpoint);
      str.assign(tempStr);
      if (!Write(str))
        return false;
    }
  }

  if (!conf.freq_outs.empty()) {
    for (auto &freq_out : conf.freq_outs) {
      if (freq_out.enable) {
        std::string str;
        char tempStr[128];

        sprintf(tempStr, "FREQUENCYOUT ENABLE %d %d %s %d",
                freq_out.pluse_width, freq_out.period, freq_out.edge.data(),
                freq_out.instance);
        str.assign(tempStr);
        if (!Write(str))
          return false;
      } else {
        std::string str;
        char tempStr[128];

        sprintf(tempStr, "FREQUENCYOUT DISABLE %d", freq_out.instance);
        str.assign(tempStr);
        if (!Write(str))
          return false;
      }
    }
  }

  if (!conf.ntrip_ports.empty()) {
    for (auto &ntrip_port : conf.ntrip_ports) {
      std::string str;
      char tempStr[128];

      sprintf(tempStr, "NTRIPCONFIG NCOM%d CLIENT %s %s %s %s %s ALL",
              ntrip_port.port, ntrip_port.protocol.data(),
              ntrip_port.endpoint.data(), ntrip_port.mountpoint.data(),
              ntrip_port.username.data(), ntrip_port.password.data());
      str.assign(tempStr);
      if (!Write(str))
        return false;
    }
  }

  if (!conf.work_freqs.empty()) {
    for (auto &work_freq : conf.work_freqs) {
      if (work_freq.system == "") {
        std::string str;
        char tempStr[128];

        sprintf(tempStr, "WORKFREQS %s", work_freq.freq.data());
        str.assign(tempStr);
        if (!Write(str))
          return false;
      } else {
        std::string str;
        char tempStr[128];

        sprintf(tempStr, "WORKFREQS %s %s", work_freq.freq.data(),
                work_freq.system.data());
        str.assign(tempStr);
        if (!Write(str))
          return false;
      }
    }
  }

  if (conf.set_pjk_flag) {
    std::string str;
    char tempStr[128];
    sprintf(tempStr, "SET PJKPARA %.3f %.3f %.9lf %.9lf %.9lf %.9lf [%.9lf %s]",
            conf.a, conf.alpha, conf.L0, conf.W0, conf.FN, conf.FE, conf.k0,
            conf.eht.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_alignment_vel) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SETALIGNMENTVEL %.3f", conf.alignment_vel);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ins_rot_flag) {
    std::string str;
    char tempStr[256];
    sprintf(tempStr, "SETINSROTATION %s %f %f %f %f %f %f", conf.ins_rot.data(),
            conf.rot_x, conf.rot_y, conf.rot_z, conf.rot_xsd, conf.rot_ysd,
            conf.rot_zsd);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ins_tran_flag) {
    std::string str;
    char tempStr[256];
    sprintf(tempStr, "SETINSTRANSLATION %s %f %f %f %f %f %f %s",
            conf.ins_tran.data(), conf.tran_x, conf.tran_y, conf.tran_z,
            conf.tran_xsd, conf.tran_ysd, conf.tran_zsd,
            conf.tran_input_frame.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ins_calibrate_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "INSCALIBRATE %s %s %f", conf.offset.data(),
            conf.trigger.data(), conf.sd_threshold);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_dns_config_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "DNSCONFIG 1 %s", conf.dns.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ip_config_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "IPCONFIG ETHA %s %s %s %s", conf.address_mode.data(),
            conf.ip_address.data(), conf.netmask.data(), conf.gateway.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ins_profile_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SETINSPROFILE %s", conf.ins_profile.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_auth_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "AUTH ADD %s", conf.auth.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_dualantenna_power_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "DUALANTENNAPOWER %s",
            (conf.dualantenna_power) ? "ON" : "OFF");
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_gps_ref_week_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "GPSREFWEEK %d", conf.gps_ref_week);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_rtk_type_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "RTKTYPE %s", conf.rtk_type.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_ecutoff_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "ECUTOFF %f", conf.ecutoff);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_base_line_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SETBASELINE ON %f %f", conf.base_line_length,
            conf.base_line_offset);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_snrcutoff_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SNRCUTOFF %f", conf.snrcutoff);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_shift_utm_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SET SHIFTDATUM %f %f %f", conf.x_offset, conf.y_offset,
            conf.z_offset);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_obs_freq_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SET OBSFREQ %d", conf.obs_freq);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_fpga_raw_freq_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "SET FPGARAWFREQ %d", conf.fpga_raw_freq);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_output_source_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "OUTPUTSOURCE %s", conf.output_source.data());
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_fix_pos_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "FIX POSITION %f %f %f", conf.fix_pos_lon,
            conf.fix_pos_lat, conf.fix_pos_alt);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_rtktimeout_flag) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "RTKTIMEOUT %d", conf.rtktimeout);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_quality_check) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "QUALITYCHECK %s %s", conf.quality_check_pos.data(),
            conf.quality_switch ? "ON" : "OFF");
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_heading_offset) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "HEADINGOFFSET %f %f", conf.heading_offsetin_deg,
            conf.pitch_offsetin_deg);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  if (conf.set_vel_period) {
    std::string str;
    char tempStr[128];

    sprintf(tempStr, "VELSMOOTH %f", conf.vel_period);
    str.assign(tempStr);
    if (!Write(str))
      return false;
  }

  return Write("SAVECONFIG");
}
} // namespace bynav_gps_driver
