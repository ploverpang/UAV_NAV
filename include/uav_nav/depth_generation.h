#ifndef UAVNAV_DEPTHGENERATION_H_
#define UAVNAV_DEPTHGENERATION_H_

#include <ros/ros.h>
#include <queue>

#include <opencv2/opencv.hpp>
#ifdef USE_GPU
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#endif

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include "uav_nav/uav_nav.h"

#define CAMERAFOV_X 60
#define CAMERAFOV_Y 56
#define CLEARANCE 2
enum representation {ONE_DIMENSIONAL = 1, TWO_DIMENSIONAL = 2};

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
void CreateDepthImage(cv::Mat& L_img, cv::Mat& R_img, cv::Mat& dst_img, int dimensionality);
void DepthProcessing(cv::Mat src_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
void maskOutliers(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& prevFrame,
				  const bool& clearCMD, const int nFrames,  const int diffThreshold);
void legacyRoundMorph(cv::Mat& src_img, int byNumber, int xy);
void roundMorph(cv::Mat& src_img, cv::Mat& dst_img, int xy, int threshold);
void dispToMeter(cv::Mat src_img, cv::Mat& dst_img);
void fovReduction(cv::Mat src_img, cv::Mat& dst_img);

#endif // UAVNAV_DEPTHGENERATION_H_
