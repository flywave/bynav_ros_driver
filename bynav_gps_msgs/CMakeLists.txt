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
  Gpgga.msg
  Gpgsa.msg
  Gpgst.msg
  Gpgsv.msg
  Gphdt.msg
  Gpori.msg
  Gprmc.msg
  Gpvtg.msg
  ICOMConfig.msg
  Insatt.msg
  Inspos.msg
  Inspva.msg
  Inspvax.msg
  Insspd.msg
  Insstdev.msg
  Insvel.msg
  Raw.msg
  RTCM.msg
  InterfaceMode.msg
  Ksxt.msg
  MarkTime.msg
  NtripPort.msg
  Pashr.msg
  BynavConfig.msg
  BynavPJK.msg
  BynavConfigQuery.msg
  BynavCorrectedImuData.msg
  BynavEnuAvr.msg
  BynavExtendedSolutionStatus.msg
  Heading.msg
  BynavMessageHeader.msg
  BynavPosition.msg
  Gpdop.msg
  BynavReceiverStatus.msg
  BynavSignalMask.msg
  BynavVelocity.msg
  PtnlPJK.msg
  Range.msg
  RangeInformation.msg
  Satellite.msg
  SerialConfig.msg
  Time.msg
  FrequencyOut.msg
  RTCM.msg
  ShiftDatum.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)
