#include "uav_nav/depth_generation.h"
#include <iostream>

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Publisher laser_scan_pub;

/* Global variables */
cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
std::string left_id, right_id;
float intensity;
// Queue to only run img processing when new images arrive.
// Can also be used to tweek parameters if queue gets too big, and it reduces overhead when using gpu.
int img_queue = 0;
float height = 2.5;

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img){
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = left_img->data[i];
	}
	cv::Mat buffer(HEIGHT, WIDTH, CV_8UC1); // This avoids artifacts that appear when modifying global variables directly
	buffer.data = arr;
	buffer.copyTo(left_img1);

	left_id = left_img->header.frame_id;
	if (left_id == "right") img_queue++;
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img){
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = right_img->data[i];
	}
	cv::Mat buffer(HEIGHT, WIDTH, CV_8UC1);
	buffer.data = arr;
	buffer.copyTo(right_img1);

	right_id = right_img->header.frame_id;
}

void CreateDepthImage(cv::Mat& L_img, cv::Mat& R_img, cv::Mat& dst_img, int dimensionality){

	if( L_img.empty() || R_img.empty() ) return;

	// For Stereo Matcher
	static int wsize =31;
	static int numDisp = 16;
	cv::Mat left_disp, masked_map, meter_map, out_img;

	// Variables keeping track of previous frame and frameID
	static std::string frame_ID_buffer;
	static cv::Mat frameBuffer_sgbm;


	#ifdef USE_GPU
	if (wsize<5) wsize = 5;
    cv::Ptr<cv::cuda::StereoBM> left_matcher = cv::cuda::createStereoBM(numDisp, wsize);
    //left_matcher->setPreFilterCap(63);
	cv::cuda::GpuMat L_cuda, R_cuda;
	L_cuda.upload(L_img);
	R_cuda.upload(R_img);
	cv::cuda::GpuMat disp_cuda(L_img.size(), CV_8UC1);
	left_matcher->compute( L_cuda, R_cuda, disp_cuda);
	disp_cuda.download(left_disp);
	// cuda StereoBM returns an 8 bit image, while the cpu implementation returns a 16 bit deep disparity
	left_disp.convertTo(left_disp, CV_16UC1, 16);
    #else
	/*cv::Ptr<cv::StereoSGBM> left_matcher  = cv::StereoSGBM::create(0,numDisp,wsize);
	// StereoSGBM parameter setup
	left_matcher->setP1(8*wsize*wsize);
	left_matcher->setP2(32*wsize*wsize);
	left_matcher->setMode(cv::StereoSGBM::MODE_SGBM);
   	left_matcher->setPreFilterCap(21);
	left_matcher->compute( L_img, R_img, left_disp);*/

	if (wsize<5) wsize = 5;
	cv::Ptr<cv::StereoBM> left_matcher  = cv::StereoBM::create(numDisp,wsize);
   	//left_matcher->setPreFilterCap(63);
	left_matcher->compute( L_img, R_img, left_disp);
	#endif


	//Masking unused borders
	int x_delta = (wsize-1)/2;
	int y_delta = (wsize-1)/2;
	int croppedWIDTH = WIDTH-numDisp-2*(x_delta);
	int croppedHEIGHT = HEIGHT-2*(y_delta);
	cv::Rect mask(numDisp + x_delta, y_delta, croppedWIDTH, croppedHEIGHT);
	left_disp = left_disp(mask);

	// Preparing disparity map for further processing
	left_disp.setTo(0, left_disp < 0);
	if (dimensionality == ONE_DIMENSIONAL){
		fovReduction(height, left_disp, left_disp);
	}

	// Dispraity map processing
	if (!frameBuffer_sgbm.empty()){
		dispToMeter(left_disp, meter_map);
		bool clearList = (frame_ID_buffer != left_id);
		maskOutliers(meter_map, masked_map, frameBuffer_sgbm, clearList, 1, 3);
		//legacyRoundMorph(masked_map, 10, 5); // might not even be needed
		DepthProcessing(masked_map);
		masked_map.copyTo(dst_img);
		//roundMorph(dst_img, dst_img, 5, 1);
	}
	else{
		//legacyRoundMorph(left_disp, 30, 5); // might not even be needed
		dispToMeter(left_disp, meter_map);
		DepthProcessing(meter_map);
		meter_map.copyTo(dst_img);
	}
	// Buffering
	meter_map.copyTo(frameBuffer_sgbm);
	frame_ID_buffer = left_id;

	// Output
	dst_img.setTo(0, dst_img > CAMERARANGE);
	dst_img.convertTo(dst_img, CV_8UC1); //Make it viewer friendly

	return;
}

void DepthProcessing(cv::Mat src_img){
	/* To avoid confusion: everything refered to as 'x' in this method corresponds to the colomns of the matrix and
	   vice versa for 'y'. Therefore when reading/writing cv::Mat values the coordinates are swapped. However as the
	   algorithm is dependent on the camera field of view, we will continue using that convention originating from
	   typical coordinate system representation. */

	// INPUT VARIABLES
	static float slice_x = 5; // width of each sector in degrees
	static float slice_y = 50; // height of each sector in degrees - > defaults to max vertical FOV

	// X and Y component of the field of view of the depth image
	const static float FOV_x = float(src_img.cols/float(WIDTH))*float(CAMERAFOV_X);
	const static float FOV_y = float(src_img.rows/float(HEIGHT))*CAMERAFOV_Y;

	// Bounding max angle to FOV
	if (slice_y > FOV_y) slice_y = FOV_y;
	if (slice_x > FOV_x) slice_x = FOV_x;

	// Computing pixel values corresponding to slice angles
	const static int numSlices_x = floor(FOV_x/slice_x);
	const static int numSlices_y = floor(FOV_y/slice_y);
	const static int slicePixelWIDTH = floor(float(src_img.cols)/(FOV_x/slice_x));
	const static int slicePixelHEIGHT = floor(float(src_img.rows)/(FOV_y/slice_y));
	int pushByPixelAmmount_x = floor((src_img.cols-(slicePixelWIDTH*numSlices_x))/2);
    int pushByPixelAmmount_y = floor((src_img.rows-(slicePixelHEIGHT*numSlices_y))/2);


    src_img.convertTo(src_img, CV_32FC1);
	float depthGridValues[numSlices_x][numSlices_y][1];
	float gridConfidence[numSlices_x][numSlices_y][1];
	for (int x_grid=0; x_grid<numSlices_x; x_grid++){
		for (int y_grid=0; y_grid<numSlices_y; y_grid++){
			float average = 0;
			int counter = 0;
			int adjustedPixelWIDTH = slicePixelWIDTH;
			int adjustedPixelHEIGHT = slicePixelHEIGHT;
			if (x_grid == 0 || x_grid == numSlices_x-1) adjustedPixelWIDTH += pushByPixelAmmount_x;
			if (y_grid == 0 || y_grid == numSlices_y-1) adjustedPixelHEIGHT += pushByPixelAmmount_y;
			for (int x_pix=0; x_pix<adjustedPixelWIDTH; x_pix++){
				for (int y_pix=0; y_pix<slicePixelHEIGHT; y_pix++){
					int x_coord = x_grid*slicePixelWIDTH + (x_grid != 0)*pushByPixelAmmount_x + x_pix;
					int y_coord = y_grid*slicePixelHEIGHT + (y_grid != 0)*pushByPixelAmmount_y + y_pix;
					float pixel_value = src_img.at<float>(y_coord, x_coord);
					if (pixel_value > 0 && pixel_value < CAMERARANGE){
						average += pixel_value;
						counter++;
					}
				}
			}
                        if (counter == 0){
                                depthGridValues[x_grid][y_grid][0] = 0;
                        }else{
                                depthGridValues[x_grid][y_grid][0] = average/counter;
                        }
			gridConfidence[x_grid][y_grid][0] = float(counter)/(adjustedPixelWIDTH*adjustedPixelHEIGHT);
		}
	}

	// ROS LaserScan message type output
	sensor_msgs::LaserScan scans;
	scans.header.frame_id = left_id;
	scans.header.stamp = ros::Time::now();
	scans.angle_min = ((slice_x-FOV_x)/2) * (M_PI/180); //negative cw
	scans.angle_max = ((FOV_x-slice_x)/2) * (M_PI/180); //positive ccw
	scans.angle_increment = slice_x * (M_PI/180);
	scans.range_min = 0;
	scans.range_max= CAMERARANGE;
	scans.ranges.resize(numSlices_x); // doesnt support multidimensional arrays
	scans.intensities.resize(numSlices_x); // doesnt support multidimensional arrays

	for (int i=0; i < numSlices_x; i++){
		if (depthGridValues[i][0][0]<=scans.range_max){
			scans.intensities[i] = gridConfidence[i][0][0];
			if (gridConfidence[i][0][0] > intensity)
				scans.ranges[i] = depthGridValues[i][0][0];
			else
				scans.ranges[i] = 0.0;
		}
	}
	laser_scan_pub.publish(scans);

	return;
}

void heightCb(const std_msgs::Float32::ConstPtr& msg)
{
  height = msg->data;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_generation_right");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh_("~");

	left_image_sub  = nh.subscribe("uav_nav/guidance/left_image",  1, left_image_callback);
	right_image_sub = nh.subscribe("uav_nav/guidance/right_image", 1, right_image_callback);
  //ros::Subscriber height_takeoff = nh.subscribe("dji_sdk/height_above_takeoff", 1, &heightCb);

	private_nh_.param("/depth_generation/intensity", intensity, 0.1f);

	laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("uav_nav/laser_scan_from_depthIMG", 1);

	cv::Mat depthMap;
	ROS_INFO("Depth generation node running");
	#ifdef USE_GPU
		ROS_INFO("with CUDA support");
	#else
		ROS_INFO("without CUDA suport");
	#endif

	while(ros::ok()) {

		if(!left_id.empty() && left_id.compare(right_id) == 0 && img_queue != 0 && left_id =="right") {  // Initial IMG rendering may delay the main loop.
			CreateDepthImage(left_img1, right_img1, depthMap, ONE_DIMENSIONAL);
			img_queue--;
			#ifndef USE_GPU
			if (!depthMap.empty()){
			imshow("Depth image in meters right (scaled by 10x)", depthMap*10);
			cv::Mat fov;
			fovReduction(height, left_img1, fov);
			imshow("fiv_right", fov);
			cv::waitKey(1);
			}
			#endif
		}
		ros::spinOnce();
	}

	return 0;
}
