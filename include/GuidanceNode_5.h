#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <opencv2/opencv.hpp>

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

#define RETURN_IF_ERR(err_code) { \
  if(err_code) { \
    release_transfer(); \
    std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; \
    return -1; \
  } \
}
