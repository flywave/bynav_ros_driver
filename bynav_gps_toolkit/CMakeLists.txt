cmake_minimum_required(VERSION 2.8.3)
project(bynav_gps_toolkit)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rosbag
    bynav_gps_driver
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(bag2rinex src/bag2rinex.cpp)
target_link_libraries(bag2rinex ${catkin_LIBRARIES})
