#include "uav_nav/sensor_feedback.h"

// Global variables
ros::Publisher                left_image_pub;
ros::Publisher                right_image_pub;
ros::Publisher                obstacle_distance_pub;
ros::Publisher                ultrasonic_pub;
geometry_msgs::Vector3Stamped rpy;

bool         show_info;                             // Show debug information
double       angle                       = 0;       // Angle by which the image is rotated (based on IMU)
double       t_y                         = 0;       // Number of pixels by which the image is translated
uint8_t      camera_select               = 0;
std::string  frame_id                    = "front"; // Indicates which sensor's image is published
e_vbus_index camera_id                   = e_vbus1; // API index to select sensor
DJI_lock     g_lock;
DJI_event    g_event;
cv::Mat      g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
cv::Mat      g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
cv::Mat      g_depth(HEIGHT,WIDTH,CV_16SC1);
cv::Mat      depth8(HEIGHT, WIDTH, CV_8UC1);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sensor_feedback");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh_("~");

  // Load parameters
  private_nh_.param("show_debug_info", show_info, false);

  //Subsriber
  ros::Subscriber rpy_sub  = nh.subscribe("uav_nav/roll_pitch_yaw",  1, &RPYCb);

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
  ROS_DEBUG("Starting Guidance data transfer");
  err_code = set_sdk_event_handler(sensorCb);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  ros::Rate r(19); // Set loop frequency at 19Hz

  while(ros::ok())
  {
    g_event.wait_event();
    private_nh_.param("show_debug_info", show_info, false);

    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;

    if(elapsed_time > ros::Duration(1)) // Publish all sides once every second
    {
      // Stop transfer in order to select new sensor
      err_code = stop_transfer();
      RETURN_IF_ERR(err_code);
      reset_config();

      switch(camera_select)
      {
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
  ROS_DEBUG("Stopping Guidance data transfer");
  err_code = stop_transfer();
  RETURN_IF_ERR(err_code);
  sleep(1); // Wait for ACK packet from Guidance
  err_code = release_transfer();
  RETURN_IF_ERR(err_code);

  return 0;
}

int sensorCb(int data_type, int data_len, char *content) // Callback to handle Guidance data inputs
{
  g_lock.enter();
  static const cv::Point2f cen(WIDTH/2, HEIGHT/2);

  // Image data
  if(e_image == data_type && NULL != content) {
    image_data* data = (image_data*)content;

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
      cv_bridge::CvImage left_8;
      g_greyscale_image_left.copyTo(left_8.image);
      left_8.header.frame_id = frame_id;
      left_8.header.stamp    = ros::Time::now();
      left_8.encoding        = sensor_msgs::image_encodings::MONO8;
      left_image_pub.publish(left_8.toImageMsg());
    }

    if(data->m_greyscale_image_right[camera_id])
    {
      memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[camera_id], WIDTH*HEIGHT);
      warpAffine(g_greyscale_image_right, g_greyscale_image_right, M_rot, cv::Size(WIDTH, HEIGHT));
      warpAffine(g_greyscale_image_right, g_greyscale_image_right, M_trans, cv::Size(WIDTH, HEIGHT));
      if(show_info)
        imshow("right", g_greyscale_image_right);

      // Publish right greyscale image
      cv_bridge::CvImage right_8;
      g_greyscale_image_right.copyTo(right_8.image);
      right_8.header.frame_id = frame_id;
      right_8.header.stamp    = ros::Time::now();
      right_8.encoding        = sensor_msgs::image_encodings::MONO8;
      right_image_pub.publish(right_8.toImageMsg());
    }

    cv::waitKey(1);
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
    g_oa.header.frame_id = frame_id;
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
    g_ul.header.frame_id = frame_id;
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

  angle = rpy.vector.x;
  t_y = rpy.vector.y*PIXEL_PER_ANGLE;
}
