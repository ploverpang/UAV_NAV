#include "uav_nav/sensor_feedback.h"

// Global variables
ros::Publisher                left_image_pub;
ros::Publisher                right_image_pub;
ros::Publisher                images_pub;
ros::Publisher                obstacle_distance_pub;
ros::Publisher                ultrasonic_pub;
geometry_msgs::Vector3Stamped rpy;

bool         show_info; // Show debug information
e_vbus_index camera_ids[3] = {e_vbus1, e_vbus2, e_vbus4};
DJI_lock     g_lock;
DJI_event    g_event;
cv::Mat      g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
cv::Mat      g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sensor_feedback", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh_("~");

  // Custom SIGINT handler
  signal(SIGINT, onShutdown);

  // Load parameters
  private_nh_.param("show_debug_info", show_info, false);

  //Subsriber
  ros::Subscriber rpy_sub  = nh.subscribe("uav_nav/roll_pitch_yaw",  1, &RPYCb);

  //Publishers
  left_image_pub			  = nh.advertise<sensor_msgs::Image>    ("uav_nav/guidance/left_image",        1);
  right_image_pub			  = nh.advertise<sensor_msgs::Image>    ("uav_nav/guidance/right_image",       1);
  images_pub    			  = nh.advertise<uav_nav::Images>       ("uav_nav/images",                     1);
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
  for(int i=0; i<CAMERA_NUM; i++)
  {
    if(online_status[i] != 1)
    {
      ROS_ERROR("Camera %i is offline. Stopping execution!", i);
      return 0;
    }
    else
      ROS_DEBUG("Camera %i is online.", i);
  }

  // Get calibration parameters
  stereo_cali cali[CAMERA_NUM];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  for(int i=0; i<CAMERA_NUM; i++)
    ROS_DEBUG("cu: %f, cv: %f, focal: %f, baseline: %f", cali[i].cu, cali[i].cv, cali[i].focal, cali[i].baseline);

  // Start data transfer
  ROS_DEBUG("Starting Guidance data transfer");
  err_code = set_sdk_event_handler(sensorCb);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);


  while(ros::ok())
  {
    g_event.wait_event();
    private_nh_.param("show_debug_info", show_info, false);
    ros::spinOnce();
  }

  return 0;
}

int sensorCb(int data_type, int data_len, char *content) // Callback to handle Guidance data inputs
{
  g_lock.enter();
  static const cv::Point2f cen(WIDTH/2, HEIGHT/2);

  // Image data
  if(e_image == data_type && NULL != content)
  {
    image_data* data = (image_data*)content;

    for(int i = 0; i < 3; ++i)
    {
      std::string frame_id = "unknown";
      double angle = 0;                       // Angle by which the image is rotated (based on IMU)
      double t_y = 0;                         // Number of pixels by which the image is translated
      e_vbus_index camera_id = camera_ids[i]; // API index to select sensor
      sensor_msgs::Image left_8, right_8;

      switch(camera_id)
      {
        case e_vbus1:
          frame_id = "front";
          angle = rpy.vector.x;
          t_y = rpy.vector.y*PIXEL_PER_ANGLE;
          break;

        case e_vbus2:
          frame_id = "right";
          angle = -rpy.vector.y;
          t_y = rpy.vector.x*PIXEL_PER_ANGLE;
          break;

        case e_vbus4:
          frame_id = "left";
          angle = rpy.vector.y;
          t_y = -rpy.vector.x*PIXEL_PER_ANGLE;
          break;

        default:
          break;
      }

      cv::Mat M_rot = getRotationMatrix2D(cen, -angle, 1);             // Rotation matrix based on IMU
      cv::Mat M_trans = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, t_y); // Translation matrix based on IMU

      if(data->m_greyscale_image_left[camera_id])
      {
        memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[camera_id], WIDTH*HEIGHT);
        warpAffine(g_greyscale_image_left, g_greyscale_image_left, M_rot, cv::Size(WIDTH, HEIGHT));
        warpAffine(g_greyscale_image_left, g_greyscale_image_left, M_trans, cv::Size(WIDTH, HEIGHT));
        if(show_info)
          imshow("left", g_greyscale_image_left);

        // Publish left greyscale image
      	left_8.header.stamp = ros::Time::now();
      	left_8.header.frame_id = frame_id;
      	left_8.height = g_greyscale_image_left.rows;
      	left_8.width = g_greyscale_image_left.cols;
      	left_8.encoding =  "mono8";
      	left_8.is_bigendian = 0;
      	left_8.step = 320;
      	left_8.data.resize(HEIGHT*WIDTH);
      	memcpy((char*)(&left_8.data[0]), g_greyscale_image_left.data, HEIGHT*WIDTH);
      	//left_image_pub.publish(left_8);
      }

      if(data->m_greyscale_image_right[camera_id])
      {
        memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[camera_id], WIDTH*HEIGHT);
        warpAffine(g_greyscale_image_right, g_greyscale_image_right, M_rot, cv::Size(WIDTH, HEIGHT));
        warpAffine(g_greyscale_image_right, g_greyscale_image_right, M_trans, cv::Size(WIDTH, HEIGHT));
        if(show_info)
          imshow("right", g_greyscale_image_right);

        // Publish right greyscale image
      	right_8.header.stamp = ros::Time::now();
      	right_8.header.frame_id = frame_id;
      	right_8.height = g_greyscale_image_right.rows;
      	right_8.width = g_greyscale_image_right.cols;
      	right_8.encoding =  "mono8";
      	right_8.is_bigendian = 0;
      	right_8.step = 320;
      	right_8.data.resize(HEIGHT*WIDTH);
      	memcpy((char*)(&right_8.data[0]), g_greyscale_image_right.data, HEIGHT*WIDTH);
      	//right_image_pub.publish(right_8);
      }

      uav_nav::Images images;
      images.header.stamp = ros::Time::now();
      images.header.frame_id = frame_id;
      images.left = left_8;
      images.right = right_8;
      images_pub.publish(images);


      //cv::waitKey(1);
    }
  }

  // Obstacle distance
  if(e_obstacle_distance == data_type && NULL != content)
  {
    obstacle_distance *oa = (obstacle_distance*)content;
    if(show_info)
    {
      ROS_INFO("frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp);
      ROS_INFO("obstacle distance: [%f, %f, %f, %f, %f]", 0.01f * oa->distance[0],
                                                          0.01f * oa->distance[1],
                                                          0.01f * oa->distance[2],
                                                          0.01f * oa->distance[3],
                                                          0.01f * oa->distance[4]);
    }

    // Publish obstacle distance
    sensor_msgs::LaserScan g_oa;
    g_oa.ranges.resize(CAMERA_NUM);
    g_oa.header.frame_id = "guidance";
    g_oa.header.stamp    = ros::Time::now();
    for(int i = 0; i < CAMERA_NUM; ++i)
      g_oa.ranges[i] = 0.01f * oa->distance[i];
    obstacle_distance_pub.publish(g_oa);
  }

  // Ultrasonic
  if(e_ultrasonic == data_type && NULL != content)
  {
    ultrasonic_data *ultrasonic = (ultrasonic_data*)content;

    if(show_info)
    {
      ROS_INFO("frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp);
      ROS_INFO("ultrasonic distance: [%f, %f, %f, %f, %f]", 0.001f * ultrasonic->ultrasonic[0],
                                                            0.001f * ultrasonic->ultrasonic[1],
                                                            0.001f * ultrasonic->ultrasonic[2],
                                                            0.001f * ultrasonic->ultrasonic[3],
                                                            0.001f * ultrasonic->ultrasonic[4]);
      ROS_INFO("reliability: [%d, %d, %d, %d, %d]", (int)ultrasonic->reliability[0],
                                                    (int)ultrasonic->reliability[1],
                                                    (int)ultrasonic->reliability[2],
                                                    (int)ultrasonic->reliability[3],
                                                    (int)ultrasonic->reliability[4]);
    }

    // Publish ultrasonic data
    sensor_msgs::LaserScan g_ul;
    g_ul.ranges.resize(CAMERA_NUM);
    g_ul.intensities.resize(CAMERA_NUM);
    g_ul.header.frame_id = "guidance";
    g_ul.header.stamp    = ros::Time::now();
    for(int d = 0; d < CAMERA_NUM; ++d)
    {
      g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
      g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
    }

    ultrasonic_pub.publish(g_ul);
  }

  g_lock.leave();
  g_event.set_event();

  return 0;
}

void RPYCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	rpy = *msg;
  rpy.vector.x = RAD2DEG(msg->vector.x);
  rpy.vector.y = RAD2DEG(msg->vector.y);
  rpy.vector.z = RAD2DEG(msg->vector.z);
}

void onShutdown(int sig)
{
  // Release data transfer
  ROS_DEBUG("Stopping Guidance data transfer");
  stop_transfer();
  sleep(1); // Wait for ACK packet from Guidance
  release_transfer();

  ros::shutdown();
}
