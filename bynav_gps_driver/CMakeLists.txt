cmake_minimum_required(VERSION 2.8.3)

project(bynav_gps_driver)

set(DEPS
  diagnostic_msgs 
  diagnostic_updater 
  gps_common
  nav_msgs 
  nodelet 
  bynav_gps_msgs
  roscpp
  sensor_msgs
  std_msgs
  swri_math_util 
  swri_roscpp
  swri_serial_util
  swri_string_util 
  tf
)

set(BUILD_DEPENDS
  ${DEPS}
  swri_nodelet
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
  

catkin_package(CATKIN_DEPENDS ${DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets
  DEPENDS Boost  
)

add_library(${PROJECT_NAME} 
  src/nodelets/bynav_gps_nodelet.cpp 
  src/bynav_connection.cpp
  src/bynav_control.cpp
  src/bynav_nmea.cpp
  src/bynav_rtcm.cpp
  src/bynav_message_extractor.cpp
  src/parsers/bestgnsspos.cpp
  src/parsers/bestpos.cpp
  src/parsers/bestvel.cpp
  src/parsers/corrimudata.cpp
  src/parsers/corrimudatas.cpp
  src/parsers/gpatr.cpp
  src/parsers/gpdop.cpp
  src/parsers/gpfpd.cpp
  src/parsers/gpgga.cpp
  src/parsers/gpgst.cpp
  src/parsers/gpgsv.cpp
  src/parsers/gphdt.cpp
  src/parsers/gphpd.cpp
  src/parsers/gpntr.cpp
  src/parsers/gpori.cpp
  src/parsers/gprmc.cpp
  src/parsers/gpvtg.cpp
  src/parsers/gpzda.cpp
  src/parsers/header.cpp
  src/parsers/heading.cpp
  src/parsers/insatt.cpp
  src/parsers/inspos.cpp
  src/parsers/inspva.cpp
  src/parsers/inspvax.cpp
  src/parsers/insspd.cpp
  src/parsers/insstdev.cpp
  src/parsers/insvel.cpp
  src/parsers/mark2time.cpp
  src/parsers/marktime.cpp
  src/parsers/parsing_utils.cpp
  src/parsers/psrvel.cpp
  src/parsers/pashr.cpp
  src/parsers/ptnlavr.cpp
  src/parsers/ptnlpjk.cpp
  src/parsers/refstation.cpp
  src/parsers/rtcm.cpp
  src/parsers/rawimu.cpp
  src/parsers/rawimus.cpp
  src/parsers/bdsephemerisb.cpp
  src/parsers/galephemerisb.cpp
  src/parsers/gloephemerisb.cpp
  src/parsers/gpsphemb.cpp
  src/parsers/qzssephemerisb.cpp
  src/parsers/rangecmpb.cpp
  src/parsers/raw_ros.cpp
  src/parsers/raw.cpp
  src/parsers/rindex.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")

add_library(${PROJECT_NAME}_nodelets 
  src/nodelets/bynav_gps_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelets
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_nodelets PROPERTIES COMPILE_FLAGS "-std=c++11")

swri_nodelet_add_node(bynav_gps_node ${PROJECT_NAME} BynavGpsNodelet)
target_link_libraries(bynav_gps_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_nodelets
)
set_target_properties(bynav_gps_node PROPERTIES COMPILE_FLAGS "-std=c++11")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(parser_tests test/parser_tests.cpp)
  target_link_libraries(parser_tests ${PROJECT_NAME})
  set_target_properties(parser_tests PROPERTIES COMPILE_FLAGS "-std=c++11")

  add_rostest_gtest(bynav_gps_tests test/bynav_gps_tests.test test/bynav_gps_tests.cpp)
  target_link_libraries(bynav_gps_tests ${PROJECT_NAME})
  set_target_properties(bynav_gps_tests PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()

install(TARGETS 
    bynav_gps_node
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
