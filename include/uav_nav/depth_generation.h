#ifndef UAVNAV_DEPTHGENERATION_H_
#define UAVNAV_DEPTHGENERATION_H_

#include <ros/ros.h>
#include <queue>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include "uav_nav/uav_nav.h"

#define CAMERAFOV_X 60
#define CAMERAFOV_Y 56
#define CLEARANCE 2

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
void CreateDepthImage(cv::Mat& L_img, cv::Mat& R_img, cv::Mat& dst_img);
void DepthProcessing(cv::Mat src_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
void maskOutliers(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& prevFrame,
				  const bool& clearCMD, const int nFrames,  const int diffThreshold);
void legacyRoundMorph(cv::Mat& src_img, int byNumber, int xy);
void roundMorph(const cv::Mat& src_img, cv::Mat dst_img, int offset, int threshold);
void dispToMeter(cv::Mat src_img, cv::Mat& dst_img);
void fovReduction(cv::Mat src_img, cv::Mat& dst_img);

#endif // UAVNAV_DEPTHGENERATION_H_
