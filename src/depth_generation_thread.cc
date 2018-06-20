#include "uav_nav/depth_generation_thread.h"

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Publisher laser_scan_pub;

/* Global variables */
int dimensionality = 1;
#ifndef USE_GPU
cv::Mat left_img1(HEIGHT, WIDTH, CV_8UC1);
#endif
/*cv::Mat right_img1(HEIGHT, WIDTH, CV_8UC1);
std::string left_id, right_id;
// Queue to only run img processing when new images arrive.
// Can also be used to tweek parameters if queue gets too big, and it reduces overhead when using gpu.
int img_queue = 0;*/
// Threading related variables

cv::Mat depthMap[nr_consumer];
pthread_mutex_t img_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t wq_mutex = PTHREAD_MUTEX_INITIALIZER;
sem_t wq_semaphore[nr_consumer];
// 3 image procesing thread + 1 work packager
pthread_t producer_id;
pthread_t consumer_id[nr_consumer];
void* status;
thread_args_t work_data[nr_consumer];
std::list<labeled_img_t>left_img_list, right_img_list;


/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img){
	labeled_img_t left_labeled;
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = left_img->data[i];	
	}
	cv::Mat buffer(HEIGHT, WIDTH, CV_8UC1); // This avoids artifacts that appear when modifying global variables directly
	buffer.data = arr;
	buffer.copyTo(left_labeled.img);
	left_labeled.id = left_img->header.frame_id;
	#ifndef USE_GPU
	if(left_labeled.id=="front")buffer.copyTo(left_img1);
	#endif
	ROS_INFO("before left_image_callback mutex");
	pthread_mutex_lock(&img_mutex);
	left_img_list.push_front(left_labeled);
	if(left_img_list.size()>nr_consumer){
		left_img_list.pop_back();
	}
	pthread_mutex_unlock(&img_mutex);
	ROS_INFO("after left_image_callback mutex");
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img){
	labeled_img_t right_labeled;
	uchar arr[HEIGHT*WIDTH];
	int size = sizeof(arr)/sizeof(*arr);
	for (int i = 0; i < size; i++){
		arr[i] = right_img->data[i];	
	}
	cv::Mat buffer(HEIGHT, WIDTH, CV_8UC1);
	buffer.data = arr;
	buffer.copyTo(right_labeled.img);

	right_labeled.id = right_img->header.frame_id;
	ROS_INFO("before right_image_callback mutex");
	pthread_mutex_lock(&img_mutex);
	right_img_list.push_front(right_labeled);
	if(right_img_list.size()>nr_consumer){
		right_img_list.pop_back();
	}
	pthread_mutex_unlock(&img_mutex);
	ROS_INFO("after right_image_callback mutex");
}

void CreateDepthImage(int thread_id, cv::Mat& L_img, cv::Mat& R_img, std::string frame_id, cv::Mat& dst_img, int dimensionality){

	if( L_img.empty() || R_img.empty() ) return;

	// For Stereo Matcher
	static int wsize =31;
	static int numDisp = 16;
	cv::Mat left_disp, masked_map, meter_map, out_img;

	// Variables keeping track of previous frame and frameID
	static cv::Mat frameBuffer_sgbm[nr_consumer];

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
		fovReduction(left_disp, left_disp);
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
			if (gridConfidence[i][0][0] > 0.1)
				scans.ranges[i] = depthGridValues[i][0][0];
			else
				scans.ranges[i] = 0.0;
		}
	}
	laser_scan_pub.publish(scans);

	return;
}

void* producer(void*){
	while(ros::ok()){
		if (left_img_list.size() != 0 && right_img_list.size() != 0)
		{
			labeled_img_t labeled_left, labeled_right;
			ROS_INFO("before producer labeled image mutex");
			pthread_mutex_lock(&img_mutex);
			labeled_left.id = left_img_list.front().id;
			labeled_right.id = right_img_list.front().id;
			left_img_list.front().img.copyTo(labeled_left.img);
			right_img_list.front().img.copyTo(labeled_right.img);
			left_img_list.pop_front();
			right_img_list.pop_front();
			pthread_mutex_unlock(&img_mutex);
			ROS_INFO("after producer labeled image mutex");

			if (labeled_left.id.compare(labeled_right.id)==0)
			{
				thread_args_t thread_args;
				thread_args.id = labeled_left.id;
				labeled_left.img.copyTo(thread_args.img_l);
				labeled_right.img.copyTo(thread_args.img_r);
				thread_args.dimension = dimensionality;
				ROS_INFO("before producer work queue mutex");
				pthread_mutex_lock(&wq_mutex);
				if (labeled_left.id.compare("front") == 0)
				{
					work_data[FRONT] = thread_args;
					sem_post(&wq_semaphore[FRONT]);
					ROS_INFO("increment work queue semaphore %i", FRONT);
				}
				if (labeled_left.id.compare("left") == 0)
				{
					work_data[LEFT] = thread_args;
					sem_post(&wq_semaphore[LEFT]);
					ROS_INFO("increment work queue semaphore %i", LEFT);
				}
				if (labeled_left.id.compare("right") == 0)
				{
					work_data[RIGHT] = thread_args;
					sem_post(&wq_semaphore[RIGHT]);
					ROS_INFO("increment work queue semaphore %i", RIGHT);
				}
				pthread_mutex_unlock(&wq_mutex);
				ROS_INFO("after producer work queue mutex");
			}
		}
		sleep(0.0001);
	}
}

void* consumer(void* thread_id_v){
	int thread_id = *((int *) thread_id_v);
	delete thread_id_v;
	ROS_INFO("thread started with ID: %i", thread_id);
	while(ros::ok()){
		ROS_INFO("waiting for work queue semaphore %i", thread_id);
		sem_wait(&wq_semaphore[thread_id]);
		ROS_INFO("received work queue semaphore %i", thread_id);
		ROS_INFO("before consumer %i work queue mutex", thread_id);
		pthread_mutex_lock(&wq_mutex);
		thread_args_t *thread_args = new thread_args_t(work_data[thread_id]);
		pthread_mutex_unlock(&wq_mutex);
		ROS_INFO("after consumer %i work queue mutex", thread_id);
		CreateDepthImage(thread_id, thread_args->img_l, thread_args->img_r, thread_args->id, thread_args->dst, thread_args->dimension);

		#ifndef USE_GPU
		thread_args->dst.copyTo(depthMap[thread_id]); // Copy depth map to output
		#endif
		delete thread_args;
		sleep(0.0001);
	}
	pthread_exit(NULL);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_generation_front");
	ros::NodeHandle nh;

	left_image_sub  = nh.subscribe("uav_nav/guidance/left_image",  1, left_image_callback);
	right_image_sub = nh.subscribe("uav_nav/guidance/right_image", 1, right_image_callback);

	laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("uav_nav/laser_scan_from_depthIMG_debug", 1);

	//cv::Mat depthMap;
	ROS_INFO("Depth generation node running");
	#ifdef USE_GPU
		ROS_INFO("with CUDA support");
	#else
		ROS_INFO("without CUDA suport");
	#endif

	for (int i=0; i<nr_consumer; i++){
		 sem_init(&wq_semaphore[i], 0, 0);
	}
	int err = pthread_create(&producer_id, NULL, producer, NULL);
	if(err){return 0;}
	for (int i = 0; i < 1; i++)
	{
		int *k = new int(i);
		int response = pthread_create(&consumer_id[i], NULL, consumer, k);
		if(response){return 0;}
	}

	while(ros::ok()) {
		#ifndef USE_GPU
		for (int i=0; i<1; i++){
		if (!depthMap[i].empty()){
		imshow("Depth image in meters front(scaled by 10x)", depthMap[i]*10);
		} 
		/*cv::Mat fov;
		fovReduction(left_img1, fov);
		imshow("fov_front,", fov);*/
		cv::Mat show = work_data[0].img_l;
		if (!show.empty()){imshow("asdas", show);}
		cv::waitKey(1);
		}
		#endif

		ros::spinOnce();
	}

	err = pthread_join(producer_id, &status);
	for (int i = 0; i < nr_consumer; i++)
	{
		int response = pthread_join(consumer_id[i], &status);
	}

	return 0;
}