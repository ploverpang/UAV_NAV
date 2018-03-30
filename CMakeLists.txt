cmake_minimum_required(VERSION 2.8.3)
project(uav_nav)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  dji_sdk
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
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${DJIOSDK_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)
add_dependencies(sensorFeedback dji_sdk_generate_messages_cpp)

add_executable(depthGeneration
  src/DepthGeneration.cpp
  src/SupportingALG.cpp
)

add_executable(histogramGrid
  src/HistogramGrid.cpp
)

target_link_libraries(histogramGrid
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)

target_link_libraries(depthGeneration
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)

add_executable(droneControl
  src/DroneControl.cpp
)
target_link_libraries(droneControl
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(droneControl dji_sdk_generate_messages_cpp)

add_executable(vfh
  src/VFH.cpp
)
target_link_libraries(vfh
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
