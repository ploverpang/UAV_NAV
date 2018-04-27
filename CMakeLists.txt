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

generate_messages(
  DEPENDENCIES
)

catkin_package()

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("-- 32bit detected")
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- 64bit detected")
    link_directories(lib/x64)
  endif ()
else()
  message("-- Non-linux platform detected but sorry we do not support :D")
endif ()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system.")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()


if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  message("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(lib/XU3)
else ()
endif ()

include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_executable(safety_interrupt
#  src/safety_interrupt.cc
#)
#target_link_libraries(safety_interrupt
#  ${catkin_LIBRARIES}
#)

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

if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, depth_generation node will be compiled with CUDA support")
else()
    message(STATUS "Did not find CUDA in the system. CPU depth_generation is used")
endif()

add_executable(depth_generation
  src/depth_generation.cc
  src/supporting_alg.cc
)
target_link_libraries(depth_generation
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )

add_executable(drone_control src/drone_control.cc)
target_link_libraries(drone_control
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES})

add_executable(vfh src/vfh.cc)
target_link_libraries(vfh
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(vfh_server src/vfh_services.cc)
target_link_libraries(vfh_server ${catkin_LIBRARIES})
add_dependencies(vfh_server uav_nav_gencpp)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
