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

add_executable(sensor_feedback
  src/sensor_feedback.cc
  src/DJI_utility.cc)
target_link_libraries(sensor_feedback
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${DJIOSDK_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)
add_dependencies(sensor_feedback dji_sdk_generate_messages_cpp)

add_executable(depth_generation
  src/depth_generation.cc
  src/supporting_alg.cc
)
target_link_libraries(depth_generation
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp
)

add_executable(drone_control
  src/drone_control.cc
)
target_link_libraries(drone_control
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
add_dependencies(drone_control dji_sdk_generate_messages_cpp)

add_executable(vfh
  src/VFH.cc
)
target_link_libraries(vfh
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
