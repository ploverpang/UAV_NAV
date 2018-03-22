#ifndef DEPTHGENERATION_HPP
#define DEPTHGENERATION_HPP

#include <ros/ros.h>
#include <queue>

#include <opencv2/ximgproc/disparity_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#define WIDTH 320
#define HEIGHT 240

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img);
cv::Mat DepthProcessing(int croppedHEIGHT, int croppedWIDTH, cv::Mat out_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, std::list<cv::Mat> &maskBuffer, int nFrames, int diffThreshold);

#endif //DEPTHGENERATION_HPP
