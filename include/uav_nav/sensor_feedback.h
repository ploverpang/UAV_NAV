#ifndef UAVNAV_SENSORFEEDBACK_H_
#define UAVNAV_SENSORFEEDBACK_H_

#include <signal.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

#include <opencv2/opencv.hpp>

#include "uav_nav/DJI_guidance.h"
#include "uav_nav/DJI_utility.h"

#include "uav_nav/uav_nav.h"

#define PIXEL_PER_ANGLE 4.2857

#define RETURN_IF_ERR(err_code) { \
  if(err_code) { \
    release_transfer(); \
    std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; \
    return -1; \
  } \
}

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value) {
  const char* s = 0;
  static char str[100]={0};
  #define PROCESS_VAL(p) case(p): s = #p; break;

  switch(value) {
    PROCESS_VAL(e_OK);
    PROCESS_VAL(e_load_libusb_err);
    PROCESS_VAL(e_sdk_not_inited);
    PROCESS_VAL(e_disparity_not_allowed);
    PROCESS_VAL(e_image_frequency_not_allowed);
    PROCESS_VAL(e_config_not_ready);
    PROCESS_VAL(e_online_flag_not_ready);
    PROCESS_VAL(e_stereo_cali_not_ready);
    PROCESS_VAL(e_libusb_io_err);
    PROCESS_VAL(e_timeout);
    default:
    strcpy(str, "Unknown error");
    s = str;
    break;
  }

  #undef PROCESS_VAL
  return out << s;
}

int sensorCb(int data_type, int data_len, char *content);
void RPYCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void onShutdown(int sig);

#endif // UAVNAV_SENSORFEEDBACK_H_
