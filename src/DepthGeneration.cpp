/*
 * depthProcessing.cpp
 *
 *  Created on: Dec, 1 2017
 *      Author: ghostfromthebottle
 */

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <queue>
#include <functional>     // std::greater

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/calib3d.hpp"

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic


ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;

ros::Publisher LaserScan_pub;

using namespace cv;
using namespace std;
#define WIDTH 320
#define HEIGHT 240

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber);
cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img, cv::Mat out_img);
void DepthProcessing(int croppedHEIGHT, int croppedWIDTH, cv::Mat out_img);

cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat imgDisparity16S(HEIGHT, WIDTH, CV_16UC1);
// Variable to change SADWindow (used to: and Morphology element size). works nice if the values are hand-in-hand. 5 and 9 look good.
const int ImgCutOff = 9;
Ptr<StereoBM> sbm = StereoBM::create(64,ImgCutOff);
int imgFlag = 0;

Ptr<CLAHE> clahe = cv::createCLAHE(4, Size(8,8));
cv::Mat leftCLAHE(HEIGHT, WIDTH, CV_8UC1);
cv::Mat rightCLAHE(HEIGHT, WIDTH, CV_8UC1);
cv::Mat dispCLAHE(HEIGHT, WIDTH, CV_16UC1);

Ptr<StereoSGBM> ssgbm = StereoSGBM::create();

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.convertTo(left_img1, CV_8UC1);
	clahe->apply(left_img1,leftCLAHE);
	std_msgs::String frameName;
	frameName.data = left_img->header.frame_id;
	ROS_INFO("%s",frameName.data.c_str());
    imgFlag++;


}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.convertTo(right_img1, CV_8UC1);
	clahe->apply(right_img1,rightCLAHE);

	imgFlag++;
}

cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img, cv::Mat out_img){

    if( L_img.empty() || R_img.empty() ){
    }
    else
    {
    // Stereo camera to depth image
    sbm->compute( L_img, R_img, out_img );
    out_img = (247.35 * 150)/out_img; //masking

    // Morphology
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(9, 9)); //11

    morphologyEx(out_img, out_img, MORPH_OPEN, element);
    morphologyEx(out_img, out_img, MORPH_CLOSE, element);

    //Masking unused borders
    int x_delta = (ImgCutOff-1)/2;
    int y_delta = (ImgCutOff-1)/2;
    int croppedWIDTH = WIDTH-63-2*(x_delta);
    int croppedHEIGHT = HEIGHT-2*(y_delta);

    cv::Rect mask(63 + x_delta, y_delta, croppedWIDTH, croppedHEIGHT);
    out_img = out_img(mask);
    out_img.convertTo(out_img, CV_8UC1);

    //DepthProcessing(croppedHEIGHT, croppedWIDTH, out_img);

    return out_img;
    }
}

void DepthProcessing(int croppedHEIGHT, int croppedWIDTH, cv::Mat out_img){

    // X and Y component of the field of view of the depth image
    float FOV_x = 45;
    float FOV_y = 45;
    float slice_x = 5; // width of each slice in degrees
    float slice_y = 45;
    float percentMin = 30 /float(100); // The bottom X percent of the grid values.


    int numSlices_x = floor(float(FOV_x/slice_x));
    int numSlices_y = floor(float(FOV_y/slice_y));
    int slicePixelWIDTH = floor(float(croppedWIDTH/numSlices_x));
    int slicePixelHEIGHT = floor(float(croppedHEIGHT/numSlices_y));
    int pushByPixelAmmount_x = floor((croppedWIDTH-(slicePixelWIDTH*numSlices_x))/2);
    int pushByPixelAmmount_y = floor((croppedHEIGHT-(slicePixelHEIGHT*numSlices_y))/2);

    // Multi dimensional array containing the average values from each slice/square of the depth image.
    float depthGidValues[numSlices_x][numSlices_y][1] = {};
    // Process non NULL values
    for (int x_grid=0; x_grid<numSlices_x; x_grid++){
        for (int y_grid=0; y_grid<numSlices_y; y_grid++){
            std::vector<int> sorted1D;
            for (int x_pix=0; x_pix<slicePixelWIDTH; x_pix++){
                for (int y_pix=0; y_pix<slicePixelHEIGHT; y_pix++){
                    if (out_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix) != 0){
                        sorted1D.push_back(out_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix));
                    }
                }
            }

            int stopNumber = sorted1D.size()*percentMin;
            int numberOfArrayElements = sorted1D.size();
            float average;

            //ROS_INFO("Start sorting");
            average = findsmallestX(sorted1D, numberOfArrayElements, stopNumber);
            //ROS_INFO("End sorting");
            //printf("\n\n");

            //ROS_INFO("The average of the bottom %f percent of the grid %ix,%iy is %f\n", percentMin*100, x_grid, y_grid, average);
            depthGidValues[x_grid][y_grid][0] = average;
            //ROS_INFO("Check for good copy x: %i, y: %i, average: %f\n", x_grid, y_grid, average);

        }
    }

    sensor_msgs::LaserScan scans;
    scans.header.frame_id = "depth scans";
    scans.header.stamp = ros::Time::now();
    scans.angle_min = ((slice_x-FOV_x)/2) * (M_PI/180);
    scans.angle_max = (FOV_x-slice_x)/2 * (M_PI/180);
    scans.angle_increment = slice_x * (M_PI/180);
    scans.time_increment = 1/20;
    scans.range_min = 20; //should be in 'm' but right now it's in 'pixel depth'
    scans.range_max= 1000;
    scans.ranges.resize(numSlices_x);
    for (int i=0; i < numSlices_x; i++){
        scans.ranges[i] = depthGidValues[i][0][0];
    }
    LaserScan_pub.publish(scans);


    //pic showing the output of this method
    cv::Mat debug(slicePixelHEIGHT*numSlices_y, slicePixelWIDTH*numSlices_x, CV_8UC1);
    printf("\n");
    for (int i=0; i<numSlices_x;i++){
        for (int j =0; j<numSlices_y; j++){
            printf("x:%i, y:%i, %f, \n", i, j, depthGidValues[i][j][0]);
            for (int ii = 0; ii < slicePixelWIDTH; ii++){
                for (int jj = 0; jj < slicePixelHEIGHT; jj++){
                    debug.at<unsigned char>(j*slicePixelHEIGHT+jj, i*slicePixelWIDTH+ii) = depthGidValues[i][j][0];
                }
            }
        }
    }
    cv:imshow("Debug pic", debug);
    cv::waitKey(1);
}

float findsmallestX(std::vector<int> arr, int numberOfArrayElements, int stopNumber) {
    float average = 0;
    if (numberOfArrayElements == 0){
        return average;
    }
    else if (stopNumber == numberOfArrayElements){ //optimization in case all values are taken into account
        for (std::vector<int>::iterator it=arr.begin(); it!=arr.end(); ++it){
            average += *it;
        }
    }
    else{
        priority_queue<int, std::vector<int>, std::greater<int> >pq;
        for (int i = 0; i < numberOfArrayElements; i++){
            pq.push(arr[i]);
        }

        for (int i = 0; i < stopNumber; i++){
            average += pq.top();
        //arr[i]=pq.top();
            pq.pop();
        }
    }
    return average/numberOfArrayElements;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

    left_image_sub        = my_node.subscribe("/rob666/guidance/left_image",  1, left_image_callback);
    right_image_sub       = my_node.subscribe("/rob666/guidance/right_image", 1, right_image_callback);

    LaserScan_pub = my_node.advertise<sensor_msgs::LaserScan>("/guidance/ocupancy_grid_values",1);

    while (ros::ok()){
    	if (imgFlag >= 2 && imgFlag%2==0){  // Initial IMG rendering delays the main loop
    		//ROS_INFO("Flag trigger: %i", imgFlag);
    		imgDisparity16S = CreateDepthImage(left_img1, right_img1, imgDisparity16S);
    		dispCLAHE = CreateDepthImage(leftCLAHE, rightCLAHE, dispCLAHE);
    		cv::imshow("Disp", imgDisparity16S);
    		cv::imshow("CLAHE disp", dispCLAHE);
    		cv::waitKey(1);
    		imgFlag = 0;
    	}

    	ros::spinOnce();
    }


    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
