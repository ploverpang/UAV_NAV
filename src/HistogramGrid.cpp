#include <DepthGeneration.hpp>
#include <tf/tf.h>

using namespace std;
using namespace cv;

ros::Subscriber laser_scan_sub;
ros::Subscriber imu_sub;
ros::Subscriber pos_sub;

// Global variables
double yaw;
cv::Mat histGrid;

void fillHistogramGrid(cv::Mat& histGrid, sensor_msgs::LaserScan msg_laser);
void shiftHistogramGrid(cv::Mat& histGrid, geometry_msgs::PointStamped msg_pos);

void imu_callback(const geometry_msgs::Vector3Stamped& msg_imu)
{
	yaw = msg_imu.vector.z;
}

void pos_callback(const geometry_msgs::PointStamped& msg_pos)
{
	shiftHistogramGrid(histGrid, msg_pos);
}

void shiftHistogramGrid(cv::Mat& histGrid, geometry_msgs::PointStamped msg_pos)
{
	static float currentPos_x = msg_pos.point.x;
	static float currentPos_y = msg_pos.point.y;
	static float hystereses = 1.2;

 	// Shift grid left, right, up, down TEST DIRECTIONS
 	if (abs(currentPos_x - msg_pos.point.x) > (hystereses*resolution_m/2))
 	{
 		int offsetX = floor((currentPos_x - msg_pos.point.x) / resolution_m);
 		Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetX, 0, 1, 0);
 		warpAffine(histGrid,histGrid,trans_mat,histGrid.size());
 		currentPos_x += offsetX; // CHECK sign
 	}
 	if (abs(currentPos_y - msg_pos.point.y) > (hystereses*resolution_m/2))
 	{
 		int offsetY = floor((currentPos_y - msg_pos.point.y) / resolution_m);
 		Mat trans_mat = (Mat_<double>(2,3) << 1, 0, 0, 0, 1, offsetY);
 		warpAffine(histGrid,histGrid,trans_mat,histGrid.size());
 		currentPos_y += offsetY; // CHECK sign
 	}
 	
 	Mat show;
	resize(histGrid, show, Size(), 10, 10, INTER_NEAREST);
	imshow("asd", show);imshow("asd", histGrid);
 	waitKey(1);
 	return;
}


void laser_scan_callback(const sensor_msgs::LaserScan& msg_laser)
{
	fillHistogramGrid(histGrid, msg_laser);
}

void fillHistogramGrid(cv::Mat& histGrid, sensor_msgs::LaserScan msg_laser)
{
	// Based on srcID, scalar times 90° is added to the yaw. CCW, Front = 0°
	std::string cameraID = msg_laser.header.frame_id;
	static int scalar; // this times 90° to rotate
	if (cameraID == "front"){scalar = 0;}
	else if (cameraID == "left"){scalar = 1;}
	else if (cameraID == "rear"){scalar = 2;}
	else if (cameraID == "right"){scalar = 3;}
	else {return;} // don't need down facing camera
	yaw *= scalar*M_PI/2;

	// Increment cell values at laserPoint with GRO mask and decrement cells along a line between center and laserPoint 
	static int increment = 3;
	static int decrement = 1;
	Point laserPoint;
	const int histCenter = (histGrid.rows-1)/2;
	// GRO filter
	static Mat_<float> kernel(3,3);
	kernel << 0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5;

	for (int i=0; i < msg_laser.ranges.size(); i++){
		if (msg_laser.ranges[i] != 0){
		laserPoint.x = histCenter + round(cos(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);
		laserPoint.y = histCenter + round(sin(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);
		
		// Increment
		if (histGrid.at<unsigned char>(laserPoint) > 255 - increment){ //Overflow protection
			histGrid.at<unsigned char>(laserPoint) = 255;
		}
		else{
		histGrid.at<unsigned char>(laserPoint) += increment;
		}

		// Applying GRO mask
		Mat onePixelSourceROI(histGrid, cv::Rect( laserPoint, cv::Size(1, 1) ));
		Mat dst (Size(1,1), CV_8UC1);
		cv::filter2D(onePixelSourceROI, dst, CV_8UC1, kernel, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT); // TEST if borders are coppied
		histGrid.at<unsigned char>(laserPoint) = dst.at<unsigned char>(0,0);
		
		// Decrement along a line
		Mat linemask = cv::Mat::zeros(histGrid.size(), CV_8UC1);
		line(linemask, Point(histCenter, histCenter), laserPoint, Scalar(255), 1, 8); // would 4 connectivity be better? TEST
		linemask.at<unsigned char>(laserPoint) = 0;
		Mat histGridDec;
		histGrid.copyTo(histGridDec);
		histGridDec -= decrement;
		histGridDec.copyTo(histGrid, linemask);
		}
	}
	Mat show;
	resize(histGrid, show, Size(), 10, 10, INTER_NEAREST);
	imshow("asd", show);
 	waitKey(1);
	return;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "histogramGrid");
	ros::NodeHandle node1;

	// Setup the histogram grid
	static int histDimension = round(float(cameraRange)*2/resolution_m);
	if (histDimension%2 !=1){histDimension++;}
	histGrid = cv::Mat::zeros(histDimension, histDimension, CV_8UC1);

	laser_scan_sub  = node1.subscribe("/rob666/guidance/laser_scan_from_depthIMG",  1, laser_scan_callback);
	imu_sub = node1.subscribe("/rob666/roll_pitch_yaw", 1, imu_callback);
	pos_sub = node1.subscribe("/dji_sdk/local_position", 1, pos_callback);

	while(ros::ok()) {
       
    ros::spinOnce();
    }

return 0;
}
