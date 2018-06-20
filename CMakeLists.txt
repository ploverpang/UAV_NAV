cmake_minimum_required(VERSION 2.8.3)
project(uav_nav)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  dji_sdk
  roslaunch
  message_generation
)
find_package(DJIOSDK REQUIRED)
find_package(OpenCV REQUIRED)

roslaunch_add_file_check(launch)

add_service_files(
  FILES
  VFHLookUpTables.srv
)

add_message_files(
  FILES
  Steering.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(lib/x64)
  endif ()
else()
  message("Platform not supported")
endif ()

find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed on the system")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA on the system")
endif()


if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  link_directories(lib/XU3)
endif ()

include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(safety_interrupt
  src/safety_interrupt.cc)
target_link_libraries(safety_interrupt
  ${catkin_LIBRARIES})

add_executable(sensor_feedback
  src/sensor_feedback.cc
  src/DJI_utility.cc)
target_link_libraries(sensor_feedback
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${DJIOSDK_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)

add_executable(depth_generation_thread
  src/depth_generation_thread.cc
  src/supporting_alg_thread.cc)
target_link_libraries(depth_generation_thread
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_executable(depth_generation_front
  src/depth_generation_front.cc
  src/supporting_alg.cc)
target_link_libraries(depth_generation_front
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_executable(depth_generation_right
  src/depth_generation_right.cc
  src/supporting_alg.cc)
target_link_libraries(depth_generation_right
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_executable(depth_generation_left
  src/depth_generation_left.cc
  src/supporting_alg.cc)
target_link_libraries(depth_generation_left
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_executable(drone_control src/drone_control.cc)
target_link_libraries(drone_control
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES})

add_executable(vfh src/vfh.cc)
target_link_libraries(vfh
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(vfh_server src/vfh_services.cc)
target_link_libraries(vfh_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
add_dependencies(vfh_server uav_nav_gencpp)

#add_executable(vfh_plot src/vfh_plot.cc)
#target_link_libraries(vfh_plot
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS})

add_executable(parameter_tuning
  src/parameter.cc
  src/supporting_alg.cc)
target_link_libraries(parameter_tuning
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
