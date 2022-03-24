cmake_minimum_required(VERSION 2.8.3)
project(bynav_gps_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_service_files(
  FILES
  BynavFRESET.srv
)

add_message_files(DIRECTORY msg FILES
  Gpatr.msg
  Gpdop.msg
  Gpfpd.msg
  Gpgga.msg
  Gpgst.msg
  Gpgsv.msg
  Gphdt.msg
  Gphpd.msg
  Gpntr.msg
  Gpori.msg
  Gprmc.msg
  Gpvtg.msg
  Gptra.msg
  Gpzda.msg
  ICOMConfig.msg
  Insatt.msg
  Inspos.msg
  Inspva.msg
  Inspvax.msg
  Insspd.msg
  Insstdev.msg
  Insvel.msg
  Raw.msg
  Rtcm.msg
  InterfaceMode.msg
  MarkTime.msg
  NtripPort.msg
  BynavConfig.msg
  PtnlPJK.msg
  PtnlAvr.msg
  BynavConfigQuery.msg
  BynavCorrectedImuData.msg
  BynavExtendedSolutionStatus.msg
  Heading.msg
  BynavPosition.msg
  Gpdop.msg
  BynavReceiverStatus.msg
  BynavSignalMask.msg
  Psrvel.msg
  PtnlPJK.msg
  Satellite.msg
  SerialConfig.msg
  Time.msg
  Pashr.msg
  FrequencyOut.msg
  RefStation.msg
  BynavVelocity.msg
  RawIMU.msg
  BynavMessageHeader.msg
  GnssEphemMsg.msg
  GnssGloEphemMsg.msg
  GnssMeasMsg.msg
  GnssObsMsg.msg
  GnssTimeMsg.msg
  WorkFREQS.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)
