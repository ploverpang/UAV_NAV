cmake_minimum_required(VERSION 2.8.3)
project(uav_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)
find_package(DJIOSDK REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
)

link_directories(lib/x64)

include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensorFeedback
  src/SensorFeedback.cpp
  src/DJI_utility.cpp)

target_link_libraries(sensorFeedback
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)

add_executable(depthGeneration
  src/DepthGeneration.cpp)

target_link_libraries(depthGeneration
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

