#ifndef UAVNAV_DEPTHGENERATION_H_
#define UAVNAV_DEPTHGENERATION_H_

#include <ros/ros.h>
#include <queue>

#include <opencv2/ximgproc/disparity_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include "uav_nav/uav_nav.h"

#define CAMERAFOV_X 60
#define CAMERAFOV_Y 56
#define CLEARANCE 2

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img);
void DepthProcessing(cv::Mat src_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, int nFrames, int diffThreshold);
cv::Mat roundMorph(cv::Mat src_img, int byNumber, int xy);
cv::Mat dispToMeter(cv::Mat src_img);
cv::Mat fovReduction(cv::Mat src_img);

#endif // UAVNAV_DEPTHGENERATION_H_
