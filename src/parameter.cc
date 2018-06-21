#include "uav_nav/depth_generation.h"

#include <fstream>

/* Global variables */
cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
std::string left_id, right_id;
float height = 0;

// Test variables
	int wsize =5;
	int numDisp = 16;
	int cap = 0;
	int buffersize = 0;
	float bufferthreshold = 1;
	int legacythreshold = 1;
	int legacysize = 1;
	int maxsize = 70800;
	cv::Mat reference, localref;

void heightCb(const std_msgs::Float32::ConstPtr& msg)
{
  height = msg->data;
}


void CreateDepthImage(cv::Mat& L_img, cv::Mat& R_img, cv::Mat& dst_img, int dimensionality){

	if( L_img.empty() || R_img.empty() ) return;

	// For Stereo Matcher
	cv::Mat left_disp, masked_map, meter_map;

	// Variables keeping track of previous frame and frameID
	static std::string frame_ID_buffer;
	static cv::Mat frameBuffer_sgbm;

	if (wsize<5) wsize = 5;
	cv::Ptr<cv::StereoBM> left_matcher  = cv::StereoBM::create(numDisp,wsize);
   	if (cap != 0) left_matcher->setPreFilterCap(cap);
	left_matcher->compute( L_img, R_img, left_disp);

	//Masking unused borders
	int x_delta = (wsize-1)/2;
	int y_delta = (wsize-1)/2;
	int croppedWIDTH = WIDTH-numDisp-2*(x_delta);
	int croppedHEIGHT = HEIGHT-2*(y_delta);
	cv::Rect mask(numDisp + x_delta, y_delta, croppedWIDTH, croppedHEIGHT);
	left_disp = left_disp(mask);

	cv::Rect refmask(numDisp -16 + x_delta - 2, y_delta - 2, croppedWIDTH, croppedHEIGHT);
	localref = reference(refmask);

	// Preparing disparity map for further processing
	left_disp.setTo(0, left_disp < 0);
	if (dimensionality == ONE_DIMENSIONAL){
		fovReduction(height, left_disp, left_disp);
	}
  dispToMeter(left_disp, meter_map);

	// Dispraity map processing
	if (!frameBuffer_sgbm.empty()){
		bool clearList = (frame_ID_buffer != left_id);
		clearList = 0;
		if (buffersize == 1) clearList = 1;
		maskOutliers(meter_map, masked_map, frameBuffer_sgbm, clearList, buffersize-1, bufferthreshold);
    masked_map.copyTo(dst_img);
	}
  else{
    meter_map.copyTo(dst_img);
  }
	// Buffering
	left_disp.copyTo(frameBuffer_sgbm);

  meter_map.copyTo(dst_img);
  dst_img.setTo(0, dst_img > CAMERARANGE);


	return;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "parameter_tuning");
	ros::NodeHandle nh;

  ros::Subscriber height_takeoff = nh.subscribe("dji_sdk/height_above_takeoff", 1, &heightCb);

	cv::Mat depthMap;
	reference = cv::imread("field_set/depth_map.png", 0);
	reference.convertTo(reference, CV_32FC1);
	reference /= 10.0;

	int counter = 1;
	int id = 1;
	int bestid_obj, bestid_dev, bestid_err, bestid_errnorm;
	float objective = 0;
	float lowestdeviation = 100.0;
	int lowesterror = 10000000;
	int lowesterror_norm = 10000000;
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    std::ofstream param_text;
  	param_text.open ("param_result.txt");
  	ros::Time starttime = ros::Time::now();
  						cv::Mat buffer;

  	wsize = 5;
  	for (int i1; wsize < 32; wsize +=2){
  		numDisp = 16;
  		for (int i2; numDisp < 65; numDisp += 16){
  			cap = 0;
  			for (int i3; cap < 64; cap +=21){
  				bufferthreshold = 1;
  				for (int i5; bufferthreshold < 10; bufferthreshold +=1){
  					buffersize = 1;
  					for (int i6; buffersize < 4; buffersize++){
  						std::string leftname = "field_set/left" + std::to_string(buffersize) + ".png";
  						std::string rightname = "field_set/right" + std::to_string(buffersize) + ".png";
  						left_img1 = cv::imread(leftname, 0);
  						right_img1 = cv::imread(rightname, 0);

  						CreateDepthImage(left_img1, right_img1, depthMap, TWO_DIMENSIONAL);
  						float deviation = fabs(cv::sum(depthMap)[0]-cv::sum(localref)[0]);
  						float coverage = float(depthMap.cols*depthMap.rows)/maxsize*100;
  						float meandeviation = deviation/depthMap.rows/depthMap.cols;

  						int errorcount = 0;
  						for (int rows = 1; rows <= depthMap.rows; rows++){
  							for (int cols = 1; cols <= depthMap.cols; cols++){
  								float value = depthMap.at<float>(rows,cols);
  								if (value != 0.0 && fabs(value-localref.at<float>(rows,cols)) > 0.5) errorcount++;
  							}
  						}

  						param_text << "With ID: " << id << " wsize: " << wsize << " numDisp: " << numDisp << " cap: " << cap << " buffersize: " << buffersize-1 << " bufferthreshold: " << bufferthreshold << "\n";
  						param_text << "coverage: " << coverage << "% mean deviation: " << meandeviation << " objective: " << (coverage / meandeviation) << " error: "<< errorcount << " error norm: "<< float(errorcount)/coverage << "\n \n";

  						if ( coverage / meandeviation > objective){
  							bestid_obj = id;
  							cv::Mat toINT;
  							depthMap.convertTo(toINT, CV_8UC1);
  							imwrite("images/depth" + std::to_string(id) + ".png", toINT*10, compression_params);
  							objective = coverage / meandeviation;
  						}

  						if ( meandeviation < lowestdeviation){
  							bestid_dev = id;
  							imwrite("images/depth" + std::to_string(id) + ".png", depthMap*10, compression_params);
  							lowestdeviation = meandeviation;
  						}

  						if ( errorcount < lowesterror){
  							bestid_err = id;
  							imwrite("images/depth" + std::to_string(id) + ".png", depthMap*10, compression_params);
  							lowesterror = errorcount;
  						}

  						if ( float(errorcount) * coverage < lowesterror_norm){
  							bestid_errnorm = id;
  							imwrite("images/depth" + std::to_string(id) + ".png", depthMap*10, compression_params);
  							lowesterror_norm = float(errorcount)/coverage;
  						}

  						ROS_INFO("id: %i", id);
  						ros::Duration(0.0001).sleep();
  						id++;
  					}
  				}
  			}
  		}
  	}

  	param_text.close();
  	ROS_INFO("highest objective score: %f, with id: %i", objective, bestid_obj);
  	ROS_INFO("lowest deviation: %f, with id: %i", lowestdeviation, bestid_dev);
  	ROS_INFO("lowest error count: %i, with id: %i", lowesterror, bestid_err);
  	ROS_INFO("lowest error norm: %i, with id: %i", lowesterror_norm, bestid_errnorm);
  	ROS_INFO("elapsed time in seconds: %f", (ros::Time::now().toSec()-starttime.toSec()));

  	return 0;
  }
