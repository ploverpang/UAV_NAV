#ifndef __DEPTHPROCESSING_H__
#define __DEPTHPROCESSING_H__

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <ros/ros.h>
#include <math.h> 
#include <vector>
#include <queue>
#include <functional>     // std::greater
#include <list>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/calib3d.hpp"
#include <opencv2/ximgproc/disparity_filter.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/stereo.hpp>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic


#define WIDTH 320
#define HEIGHT 240

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img, cv::Mat out_img);
cv::Mat DepthProcessing(int croppedHEIGHT, int croppedWIDTH, cv::Mat out_img);
void show_histogram(std::string const& name, cv::Mat1b const& image);
cv::Mat maskOutliers(cv::Mat src_img, cv::Mat prevFrame, std::list<cv::Mat> &maskBuffer, int nFrames, int diffThreshold);

#endif
