#ifndef DEPTHGENERATION_HPP
#define DEPTHGENERATION_HPP

#include <ros/ros.h>
#include <queue>

#include <opencv2/ximgproc/disparity_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#define WIDTH 320
#define HEIGHT 240
#define cameraFOVx 60
#define cameraFOVy 56
#define cameraRange 10
#define clearance 2
#define resolution_m 0.5

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img);
void DepthProcessing(cv::Mat src_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, int nFrames, int diffThreshold);
cv::Mat roundMorph(cv::Mat src_img, int byNumber, int xy);
cv::Mat dispToMeter(cv::Mat src_img);
cv::Mat fovReduction(cv::Mat src_img);


#endif //DEPTHGENERATION_HPP
