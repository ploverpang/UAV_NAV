#ifndef UAVNAV_DEPTHGENERATION_H_
#define UAVNAV_DEPTHGENERATION_H_

#include <ros/ros.h>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

#include <opencv2/opencv.hpp>
#ifdef USE_GPU
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#endif

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include <uav_nav/Images.h>
#include <std_msgs/Float32.h>

#include "uav_nav/uav_nav.h"

#define CAMERAFOV_X 60
#define CAMERAFOV_Y 56
#define CLEARANCE 2
#define NR_CONSUMER 3
enum representation {ONE_DIMENSIONAL = 1, TWO_DIMENSIONAL = 2};
enum camera_source {FRONT = 0, LEFT = 1, RIGHT = 2}; 
struct thread_args_t{
		std::string id; 
		cv::Mat img_l;
		cv::Mat img_r;
		cv::Mat dst;
};

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
void CreateDepthImage(int thread_id, cv::Mat& L_img, cv::Mat& R_img, std::string frame_id, cv::Mat& dst_img);
void DepthProcessing(cv::Mat src_img, std::string frame_id);
void show_histogram(std::string const& name, cv::Mat1b const& image);
void maskOutliers(int thread_id, const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& prevFrame,
				  const int nFrames,  const float diffThreshold);
void legacyRoundMorph(cv::Mat& src_img, int byNumber, int xy);
void roundMorph(cv::Mat& src_img, cv::Mat& dst_img, int xy, int threshold);
void dispToMeter(cv::Mat src_img, cv::Mat& dst_img);
void fovReduction(float alt, cv::Mat src_img, cv::Mat& dst_img);

#endif // UAVNAV_DEPTHGENERATION_H_
