#include "uav_nav/depth_generation_thread.h"

ros::Subscriber image_sub;
ros::Subscriber altitude_takeoff;
ros::Publisher laser_scan_pub;

/* Global variables */
int dimensionality = 1;
float intensity;
float altitude = 2.5;
cv::Mat front_img(HEIGHT, WIDTH, CV_8UC1);
// Threading related variables
cv::Mat depthMap[NR_CONSUMER];
pthread_mutex_t img_mutex[NR_CONSUMER];
sem_t wq_semaphore[NR_CONSUMER];
pthread_t consumer_id[NR_CONSUMER];
thread_args_t work_data[NR_CONSUMER];


/* left greyscale image */
void image_callback(const uav_nav::Images img){
	const int size = HEIGHT*WIDTH;
	uchar arr_l[size], arr_r[size];
	for (int i = 0; i < size; i++){
		arr_l[i] = img.left.data[i];
		arr_r[i] = img.right.data[i];	
	}
	cv::Mat buffer_l(HEIGHT, WIDTH, CV_8UC1); // This avoids artifacts that appear when modifying global variables directly
	cv::Mat buffer_r(HEIGHT, WIDTH, CV_8UC1); // This avoids artifacts that appear when modifying global variables directly
	buffer_l.data = arr_l;
	buffer_r.data = arr_r;
	thread_args_t thread_args;
	buffer_l.copyTo(thread_args.img_l);
	buffer_r.copyTo(thread_args.img_r);
	thread_args.id = img.header.frame_id;

	int sem_value;
	if (thread_args.id.compare("front")==0){
		if (pthread_mutex_trylock(&img_mutex[FRONT]) == 0){
			work_data[FRONT] = thread_args;
			sem_getvalue(&wq_semaphore[FRONT], &sem_value);
			if (!sem_value) sem_post(&wq_semaphore[FRONT]);
			pthread_mutex_unlock(&img_mutex[FRONT]);
		}
		#ifndef USE_GPU
		thread_args.img_l.copyTo(front_img);
		#endif
	}
	if (thread_args.id.compare("left")==0){
		if (pthread_mutex_trylock(&img_mutex[LEFT]) == 0){
			work_data[LEFT] = thread_args;
			sem_getvalue(&wq_semaphore[LEFT], &sem_value);
			if (!sem_value) sem_post(&wq_semaphore[LEFT]);
			pthread_mutex_unlock(&img_mutex[LEFT]);
		}
	}
	if (thread_args.id.compare("right")==0){
		if (pthread_mutex_trylock(&img_mutex[RIGHT]) == 0){
			work_data[RIGHT] = thread_args;
			sem_getvalue(&wq_semaphore[RIGHT], &sem_value);
			if (!sem_value) sem_post(&wq_semaphore[RIGHT]);
			pthread_mutex_unlock(&img_mutex[RIGHT]);
		}
	}
}


void CreateDepthImage(int thread_id, cv::Mat& L_img, cv::Mat& R_img, std::string frame_id, cv::Mat& dst_img){

	if( L_img.empty() || R_img.empty() ) return;

	// For Stereo Matcher
	static int wsize =31;
	static int numDisp = 16;
	cv::Mat left_disp, masked_map, meter_map, out_img;

	// Variables keeping track of previous frame and frameID
	static cv::Mat frameBuffer_sgbm[NR_CONSUMER];

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
	left_disp.convertTo(left_disp, CV_16UC1); //1/16?
    #else
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
		fovReduction(CLEARANCE, left_disp, left_disp);
		maskGround(altitude, left_disp, left_disp);
	}

	// Dispraity map processing
	if (!frameBuffer_sgbm[thread_id].empty()){
		dispToMeter(left_disp, meter_map);
		maskOutliers(thread_id, meter_map, masked_map, frameBuffer_sgbm[thread_id], 1, 3);
		DepthProcessing(masked_map, frame_id);
		masked_map.copyTo(dst_img);
	}
	else{
		dispToMeter(left_disp, meter_map);
		DepthProcessing(meter_map, frame_id);
		meter_map.copyTo(dst_img);
	}
	// Buffering
	meter_map.copyTo(frameBuffer_sgbm[thread_id]);

	// Output
	dst_img.setTo(0, dst_img > CAMERARANGE);
	dst_img.convertTo(dst_img, CV_8UC1); //Make it viewer friendly

	return;
}

void DepthProcessing(cv::Mat src_img, std::string frame_id){
	/* To avoid confusion: everything refered to as 'x' in this method corresponds to the colomns of the matrix and 
	   vice versa for 'y'. Therefore when reading/writing cv::Mat values the coordinates are swapped. However as the
	   algorithm is dependent on the camera field of view, we will continue using that convention originating from 
	   typical coordinate system representation. */ 

	// INPUT VARIABLES
	static float slice_x = 5; // width of each sector in degrees
	static float slice_y = 50; // height of each sector in degrees - > defaults to max vertical FOV

	// X and Y component of the field of view of the depth image
	const static float FOV_x = float(src_img.cols/float(WIDTH))*float(CAMERAFOV_X);
	const static float FOV_y = float(src_img.rows/float(HEIGHT))*float(CAMERAFOV_Y);

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
			int debug_counter = 0;
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
			}			gridConfidence[x_grid][y_grid][0] = float(counter)/(adjustedPixelWIDTH*adjustedPixelHEIGHT);
		}
	}

	// ROS LaserScan message type output
	sensor_msgs::LaserScan scans;
	scans.header.frame_id = frame_id;
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

void* consumer(void* thread_id_v){
	int thread_id = *((int *) thread_id_v);
	delete (int*)thread_id_v;
	while(ros::ok()){
		sem_wait(&wq_semaphore[thread_id]);
		pthread_mutex_lock(&img_mutex[thread_id]);
		thread_args_t *thread_args = new thread_args_t(work_data[thread_id]);
		pthread_mutex_unlock(&img_mutex[thread_id]);
		CreateDepthImage(thread_id, thread_args->img_l, thread_args->img_r, thread_args->id, thread_args->dst);

		#ifndef USE_GPU
		thread_args->dst.copyTo(depthMap[thread_id]); // Copy depth map to output
		#endif
		delete thread_args;
		sleep(0.0001);
	}
	pthread_exit(NULL);
}

void altitudeCb(const std_msgs::Float32::ConstPtr& msg)
{
  altitude = msg->data;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_generation_front");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh_("~");

	image_sub = nh.subscribe("uav_nav/images", 3, image_callback);
	//altitude_takeoff = nh.subscribe("dji_sdk/height_above_takeoff", 1, &altitudeCb);

	private_nh_.param("/depth_generation/intensity", intensity, 0.1f);

	laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("uav_nav/laser_scan_from_depthIMG_debug", 1);

	//cv::Mat depthMap;
	ROS_INFO("Depth generation node running");
	#ifdef USE_GPU
		ROS_INFO("with CUDA support");
	#else
		ROS_INFO("without CUDA suport");
	#endif

	for (int i=0; i<NR_CONSUMER; i++){
		 sem_init(&wq_semaphore[i], 0, 0);
		 img_mutex[i] = PTHREAD_MUTEX_INITIALIZER;
	}
	for (int i = 0; i < NR_CONSUMER; i++)
	{
		int *k = new int(i);
		int response = pthread_create(&consumer_id[i], NULL, consumer, k);
		if(response){return 0;}
	}

	while(ros::ok()) {
		#ifndef USE_GPU
		for (int i=0; i<NR_CONSUMER; i++){
			if (!depthMap[i].empty()){
				std::string name = "Depth image in meters front(scaled by 10x)" + std::to_string(i);
				imshow(name, depthMap[i]*10);
			}
		}
		cv::Mat fov;
		fovReduction(CLEARANCE, front_img, fov);
		maskGround(altitude, fov, fov);
		imshow("fov_front,", fov);
		cv::waitKey(1);
		#endif

		ros::spinOnce();
	}

	return 0;
}