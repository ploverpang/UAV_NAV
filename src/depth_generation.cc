#include "uav_nav/depth_generation.h"

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Publisher laser_scan_pub;

cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);

//Global variables
std::string left_id, right_id;


/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img){
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = left_img->data[i];	
	}
	left_img1.data = arr;
	
	left_id = left_img->header.frame_id;
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img){
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = right_img->data[i];	
	}
	right_img1.data = arr;

	right_id = right_img->header.frame_id;
}

void CreateDepthImage(cv::Mat& L_img, cv::Mat& R_img, cv::Mat& dst_img){

	if( L_img.empty() || R_img.empty() ) return;

	// For StereoSGBM
	const static int wsize =3;
	const static int numDisp = 64;
	static cv::Ptr<cv::StereoSGBM> sgbm  = cv::StereoSGBM::create(0,numDisp,wsize);
	cv::Mat left_sgbm, masked_sgbm, out_img;

	// Variables keeping track of previous frame and frameID
	static std::string frame_ID_buffer;
	static cv::Mat frameBuffer_sgbm;

	// StereoSGBM parameter setup
	sgbm->setP1(8*wsize*wsize);
	sgbm->setP2(32*wsize*wsize);
	sgbm->setPreFilterCap(21);
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
	sgbm->compute( L_img, R_img, left_sgbm);

	//Masking unused borders
	int x_delta = (wsize-1)/2;
	int y_delta = (wsize-1)/2;
	int croppedWIDTH = WIDTH-numDisp-2*(x_delta);
	int croppedHEIGHT = HEIGHT-2*(y_delta);
	cv::Rect mask(numDisp + x_delta, y_delta, croppedWIDTH, croppedHEIGHT);
	left_sgbm = left_sgbm(mask);

	// Preparing disparity map for further processing
	left_sgbm.convertTo(left_sgbm, CV_8UC1);
	left_sgbm.setTo(0, left_sgbm==255);
	fovReduction(left_sgbm, left_sgbm);

	// Dispraity map processing
	if (!frameBuffer_sgbm.empty()){
		bool clearList = (frame_ID_buffer != left_id);
		maskOutliers(left_sgbm, masked_sgbm, frameBuffer_sgbm, clearList, 1, 15);
		legacyRoundMorph(masked_sgbm, 30, 5); // might not even be needed
		cv::Mat float_mat;
		dispToMeter(masked_sgbm, float_mat);
		DepthProcessing(float_mat);
		masked_sgbm.copyTo(dst_img);
	}
	else{
		legacyRoundMorph(left_sgbm, 30, 5); // might not even be needed
		dispToMeter(left_sgbm, left_sgbm);
		DepthProcessing(left_sgbm);
		left_sgbm.copyTo(dst_img);
	}
	// Buffering
	left_sgbm.copyTo(frameBuffer_sgbm);
	frame_ID_buffer = left_id;

	// Output
	dst_img.convertTo(dst_img, CV_8UC1); //Make it viewer friendly

	return;
}

void DepthProcessing(cv::Mat src_img){

	// X and Y component of the field of view of the depth image
	const static float FOV_x = float(src_img.cols/float(WIDTH))*float(CAMERAFOV_X);
	const static float FOV_y = float(src_img.rows/float(HEIGHT))*CAMERAFOV_Y;
	static float slice_x = 5; // width of each slice in degrees
	static float slice_y = 45; // has to be equal or lower than FOV_y

	// Bounding max angle to FOV
	if (slice_y > FOV_y){
		slice_y = FOV_y;
	}else if (slice_x > FOV_x){
		slice_x = FOV_x;
	}

	// Computing pixel values corresponding to slice angles
	const static int numSlices_x = floor(float(FOV_x/slice_x));
	const static int numSlices_y = floor(float(FOV_y/slice_y));
	const static int slicePixelWIDTH = floor(float(src_img.cols)/numSlices_x);
	const static int slicePixelHEIGHT = floor(float(src_img.rows)/numSlices_y);

	float depthGridValues[numSlices_x][numSlices_y][1] = {};
	for (int x_grid=0; x_grid<numSlices_x; x_grid++){
		for (int y_grid=0; y_grid<numSlices_y; y_grid++){
			float average = 0;
			int counter = 0;
			for (int x_pix=0; x_pix<slicePixelWIDTH; x_pix++){
				for (int y_pix=0; y_pix<slicePixelHEIGHT; y_pix++){
					if (src_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix) != 0 && src_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix) < CAMERARANGE){
						average += src_img.at<unsigned char>(y_grid*slicePixelHEIGHT+y_pix, x_grid*slicePixelWIDTH+x_pix);
						counter++;
					}
				}
			}
			depthGridValues[x_grid][y_grid][0] = average/counter;
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
	scans.ranges.resize(numSlices_x);
	for (int i=0; i < numSlices_x; i++){
		if (depthGridValues[i][0][0]<=scans.range_max){
			scans.ranges[i] = depthGridValues[i][0][0];
		}
	}
	laser_scan_pub.publish(scans);

	return;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_generation");
	ros::NodeHandle nh;

	left_image_sub  = nh.subscribe("uav_nav/guidance/left_image",  1, left_image_callback);
	right_image_sub = nh.subscribe("uav_nav/guidance/right_image", 1, right_image_callback);

	laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("uav_nav/laser_scan_from_depthIMG", 1);

	cv::Mat depthMap;
	while(ros::ok()) {
		if(!left_id.empty() && left_id.compare(right_id) == 0) {  // Initial IMG rendering may delay the main loop
			CreateDepthImage(left_img1, right_img1, depthMap);
			if (!depthMap.empty()){
			cv::imshow("Depth image in meters (scaled by 10x)", depthMap);
			cv::waitKey(1);
			}
		}
		ros::spinOnce();
	}

	return 0;
}
