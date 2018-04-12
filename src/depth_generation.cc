#include "uav_nav/depth_generation.h"

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Publisher laser_scan_pub;

cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
cv::Mat frameBuffer_sgbm;


// For StereoSGBM
const int wsize =3;
const int numDisp = 64;
cv::Ptr<cv::StereoSGBM> sgbm  = cv::StereoSGBM::create(0,numDisp,wsize);

//Other variables
int imgFlag = 0;
std::string left_id, right_id;


/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	left_id = left_img->header.frame_id;
	cv_ptr->image.convertTo(left_img1, CV_8UC1);
	imgFlag++;
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	right_id = right_img->header.frame_id;
	cv_ptr->image.convertTo(right_img1, CV_8UC1);
	imgFlag++;
}

cv::Mat CreateDepthImage(cv::Mat L_img, cv::Mat R_img){

	if( L_img.empty() || R_img.empty() ){
	}
	else
	{
		cv::Mat left_sgbm, masked_sgbm, out_img;

		// StereoSGBM
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
		left_sgbm.convertTo(left_sgbm, CV_8UC1);
		left_sgbm.setTo(0, left_sgbm==255);
		fovReduction(left_sgbm, left_sgbm);

		if (!frameBuffer_sgbm.empty()){
			maskOutliers(left_sgbm, masked_sgbm, frameBuffer_sgbm, 1, 15);  // fix frame_id issue
			legacyRoundMorph(masked_sgbm, 15, 5); // might not even be needed
			cv::Mat float_mat = dispToMeter(masked_sgbm);
			DepthProcessing(float_mat);
			out_img = masked_sgbm	;
		}else{
			left_sgbm = dispToMeter(left_sgbm);
			DepthProcessing(left_sgbm);
			out_img = left_sgbm;
		}
		left_sgbm.copyTo(frameBuffer_sgbm);

		out_img.convertTo(out_img, CV_8UC1); //Make it viewer friendly
		return out_img;
	}
}

void DepthProcessing(cv::Mat src_img){

	// X and Y component of the field of view of the depth image
	const static float FOV_x = float(src_img.cols/float(WIDTH))*float(CAMERAFOV_X);
	const static float FOV_y = float(src_img.rows/float(HEIGHT))*CAMERAFOV_Y;
	static float slice_x = 5; // width of each slice in degrees
	static float slice_y = 45; // has to be equal or lower than FOV_y

	if (slice_y > FOV_y){
		slice_y = FOV_y;
	}else if (slice_x > FOV_x){
		slice_x = FOV_x;
	}
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

	while(ros::ok()) {
		if(imgFlag >= 2 && imgFlag%2==0 && left_id.compare(right_id) == 0) {  // Initial IMG rendering delays the main loop
			cv::Mat depthMap = CreateDepthImage(left_img1, right_img1);
			cv::imshow("To meter", depthMap);
			cv::waitKey(1);
			imgFlag = 0;
		}

		ros::spinOnce();
	}

	return 0;
}
