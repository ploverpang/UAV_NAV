#include "uav_nav/DJI_guidance.h"
#include "uav_nav/DJI_utility.h"
#include "uav_nav/sensor_feedback.h"

ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher ultrasonic_pub;

geometry_msgs::Vector3Stamped    rpy;

double       angle                       = 0;
double       t_y                         = 0;
bool         show_info                   = true;
uint8_t      camera_select               = 0;
std::string  frame_id                    = "front";
e_vbus_index camera_id                   = e_vbus1;
DJI_lock     g_lock;
DJI_event    g_event;
cv::Mat      g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
cv::Mat      g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
cv::Mat      g_depth(HEIGHT,WIDTH,CV_16SC1);
cv::Mat      depth8(HEIGHT, WIDTH, CV_8UC1);
cv::Point2f  cen(WIDTH/2, HEIGHT/2);

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "sensor_feedback");
  ros::NodeHandle nh;

  //Subsriber
  ros::Subscriber rpy_sub  = nh.subscribe("uav_nav/roll_pitch_yaw",  1, &rpy_callback);

  //Publishers
  left_image_pub			  = nh.advertise<sensor_msgs::Image>    ("uav_nav/guidance/left_image",        1);
  right_image_pub			  = nh.advertise<sensor_msgs::Image>    ("uav_nav/guidance/right_image",       1);
  obstacle_distance_pub	= nh.advertise<sensor_msgs::LaserScan>("uav_nav/guidance/obstacle_distance", 1);
  ultrasonic_pub			  = nh.advertise<sensor_msgs::LaserScan>("uav_nav/guidance/ultrasonic",        1);

  // Initialize Guidance
  reset_config();
  int err_code = init_transfer();
  RETURN_IF_ERR(err_code);

  // Check sensor online status
  int online_status[CAMERA_NUM];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  std::cout<<"Sensor online status: ";
  for(int i=0; i<CAMERA_NUM; i++) {
    std::cout<<online_status[i]<<" ";
  }
  std::cout<<std::endl;

  // Get calibration parameters
  stereo_cali cali[CAMERA_NUM];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  std::cout<<"cu\tcv\tfocal\tbaseline\n";
  for(int i=0; i<CAMERA_NUM; i++) {
    std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
  }

  // Select data
  err_code = select_greyscale_image(camera_id, true); //Left
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(camera_id, false); //Right
  RETURN_IF_ERR(err_code);
  err_code = select_depth_image(camera_id);
  RETURN_IF_ERR(err_code);
  select_ultrasonic();
  select_obstacle_distance();

  // Start data transfer
  std::cout<<"Starting data transfer"<<std::endl;
  err_code = set_sdk_event_handler(sensor_callback);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  ros::Rate r(19); //19Hz

  while (ros::ok()) {
    g_event.wait_event();

    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;

    if(elapsed_time > ros::Duration(1)) { //Publish all sides once every second
      // Stop transfer in order to select new sensor
      err_code = stop_transfer();
      RETURN_IF_ERR(err_code);
      reset_config();

      switch (camera_select) {
        case 0:
          camera_id = e_vbus2;
          frame_id = "right";
          angle = rpy.vector.y;
          t_y = rpy.vector.x*PIXEL_PER_ANGLE;
          camera_select = 1;
          break;
        case 1:
          camera_id = e_vbus3;
          frame_id = "rear";
          angle = rpy.vector.x;
          t_y = rpy.vector.y*PIXEL_PER_ANGLE;
          camera_select = 2;
          break;
        case 2:
          camera_id = e_vbus4;
          frame_id = "left";
          angle = rpy.vector.y;
          t_y = rpy.vector.x*PIXEL_PER_ANGLE;
          camera_select = 3;
          break;
        case 3:
          camera_id = e_vbus1;
          frame_id = "front";
          angle = rpy.vector.x;
          t_y = rpy.vector.y*PIXEL_PER_ANGLE;
          camera_select = 0;
          start_time = ros::Time::now();
          break;
      }

      // Select data
      err_code = select_greyscale_image(camera_id, true); //Left
      RETURN_IF_ERR(err_code);
      err_code = select_greyscale_image(camera_id, false); //Right
      RETURN_IF_ERR(err_code);
      select_ultrasonic();
      select_obstacle_distance();

      // Start data transfer
      err_code = start_transfer();
      RETURN_IF_ERR(err_code);
    }

    ros::spinOnce();
    r.sleep();
  }

  // Release data transfer
  std::cout << "Stopping transfer" << std::endl;
  err_code = stop_transfer();
  RETURN_IF_ERR(err_code);
  sleep(1); // Wait for ACK packet from Guidance
  err_code = release_transfer();
  RETURN_IF_ERR(err_code);

  return 0;
}

int sensor_callback(int data_type, int data_len, char *content) {
  g_lock.enter();

  // Image data
  if(e_image == data_type && NULL != content) {
    image_data* data = (image_data*)content;

    cv::Mat M_rot = getRotationMatrix2D(cen, angle, 1);
    cv::Mat M_trans = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, t_y);

    if(data->m_greyscale_image_left[camera_id]) {
      memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[camera_id], IMAGE_SIZE);
      warpAffine(g_greyscale_image_left, g_greyscale_image_left, M_rot, cv::Size(WIDTH, HEIGHT));
      warpAffine(g_greyscale_image_left, g_greyscale_image_left, M_trans, cv::Size(WIDTH, HEIGHT));
      if(show_info) imshow("left",  g_greyscale_image_left);

      // Publish left greyscale image
      cv_bridge::CvImage left_8;
      g_greyscale_image_left.copyTo(left_8.image);
      left_8.header.frame_id = frame_id;
      left_8.header.stamp = ros::Time::now();
      left_8.encoding = sensor_msgs::image_encodings::MONO8;
      left_image_pub.publish(left_8.toImageMsg());
    }

    if(data->m_greyscale_image_right[camera_id]) {
      memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[camera_id], IMAGE_SIZE);
      warpAffine(g_greyscale_image_right, g_greyscale_image_right, M_rot, cv::Size(WIDTH, HEIGHT));
      warpAffine(g_greyscale_image_left, g_greyscale_image_left, M_trans, cv::Size(WIDTH, HEIGHT));
      if(show_info) imshow("right", g_greyscale_image_right);

      // Publish right greyscale image
      cv_bridge::CvImage right_8;
      g_greyscale_image_right.copyTo(right_8.image);
      right_8.header.frame_id = frame_id;
      right_8.header.stamp = ros::Time::now();
      right_8.encoding = sensor_msgs::image_encodings::MONO8;
      right_image_pub.publish(right_8.toImageMsg());
    }

    cv::waitKey(1);
  }

  // Obstacle distance
  if(e_obstacle_distance == data_type && NULL != content) {
    obstacle_distance *oa = (obstacle_distance*)content;
    if(show_info) {
      printf("frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp);
      printf("obstacle distance:");
      for(int i = 0; i < CAMERA_NUM; ++i) {
        printf(" %f ", 0.01f * oa->distance[i]);
      }
      printf( "\n" );
    }

    // Publish obstacle distance
    sensor_msgs::LaserScan g_oa;
    g_oa.ranges.resize(CAMERA_NUM);
    g_oa.header.frame_id = frame_id;
    g_oa.header.stamp = ros::Time::now();
    for(int i = 0; i < CAMERA_NUM; ++i) {
      g_oa.ranges[i] = 0.01f * oa->distance[i];
    }
    obstacle_distance_pub.publish(g_oa);
  }

  // Ultrasonic
  if(e_ultrasonic == data_type && NULL != content) {
    ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
    if(show_info) {
      printf("frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp);
      for(int d = 0; d < CAMERA_NUM; ++d) {
        printf("ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d]);
      }
    }

    // Publish ultrasonic data
    sensor_msgs::LaserScan g_ul;
    g_ul.ranges.resize(CAMERA_NUM);
    g_ul.intensities.resize(CAMERA_NUM);
    g_ul.header.frame_id = frame_id;
    g_ul.header.stamp = ros::Time::now();
    for(int d = 0; d < CAMERA_NUM; ++d) {
      g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
      g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
    }
    ultrasonic_pub.publish(g_ul);
  }

  g_lock.leave();
  g_event.set_event();

  return 0;
}

void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	rpy = *msg;
}
