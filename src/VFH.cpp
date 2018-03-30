#include "VFH.hpp"
#include "DepthGeneration.hpp"

// Inputs
const int s = 36;       // Number of angular sectors
const float t_high = 0.6;
const float t_low = 0.4;
float r_enl=1;
double mu[3] = {5,2,2}; // Cost parameters
double target[2] = {10, 10}; // Target for the drone

// Vars
ros::Publisher                vel_cmd_pub;
geometry_msgs::PointStamped   local_position;
geometry_msgs::Vector3Stamped rpy;                       // Roll, pitch, yaw
geometry_msgs::Vector3Stamped velocity;
const unsigned                alpha           = 360 / s; // Sector angle
const unsigned                s_max           = 16;      // Min number of sectors for a wide valley
double                        k_d             = 0.0;     // Direction of motion
double                        prev_k_d        = 0.0;     // Previous direction of motion
double                        k_target        = 0.0;     // Target direction
std::vector<int>              k_r;                       // Right borders
std::vector<int>              k_l;                       // Left borders
std::vector<double>           c;                         // Candidate directions
float h[s] = {0};
float prev_h[s] = {0};
float *beta;
float *dist_scaled;
float *enlarge;
cv::Mat hist_grid;
unsigned masked_hist[s] = {0};

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "vfh");
  ros::NodeHandle nh;

  // Setup the histogram hist_grid
  static int histDimension = round(float(cameraRange)*2/resolution_m);
  if (histDimension%2 !=1){histDimension++;}
  hist_grid = cv::Mat::zeros(histDimension, histDimension, CV_8UC1);
  beta = new float [hist_grid.rows * hist_grid.cols];
  dist_scaled = new float [hist_grid.rows * hist_grid.cols];
  enlarge = new float [hist_grid.rows * hist_grid.cols];
  generate_LUTs();

  //Subsriber
  ros::Subscriber loc_pos_sub = nh.subscribe("dji_sdk/local_position", 1, &local_position_callback);
  ros::Subscriber vel_sub     = nh.subscribe("dji_sdk/velocity",       1, &velocity_callback);
  ros::Subscriber rpy_sub     = nh.subscribe("rob666/roll_pitch_yaw",  1, &rpy_callback);
  ros::Subscriber laser_scan_sub  = nh.subscribe("/rob666/guidance/laser_scan_from_depthIMG",  1, laser_scan_callback);

  //Publishers
  vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped> ("/rob666/vel_cmd", 1);

  //ros::Rate r(1); //1Hz

  ROS_INFO("GOOD");

  while (ros::ok()) {
    binary_hist();
    masked_polar_histogram();
    get_borders();
    get_candidates();
    calc_cost();
    publish_cmd();

    ROS_INFO("");
    ros::spinOnce();
    //r.sleep();
  }

  return 0;
}

// Callbacks
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  local_position = *msg;

  double target_angle = std::atan2(target[1] - local_position.point.y, target[0] - local_position.point.x);
  double angle_diff = target_angle - rpy.vector.z;
  angle_diff = wrap_to_pi(angle_diff);
  k_target = RAD2DEG(angle_diff) / alpha;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  velocity = *msg;
}

void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  rpy = *msg;
}

void pos_callback(const geometry_msgs::PointStamped& msg_pos) {
  shiftHistogramGrid(msg_pos);
}

void laser_scan_callback(const sensor_msgs::LaserScan& msg_laser) {
  fillHistogramGrid(msg_laser);
}

// Functions
void shiftHistogramGrid(geometry_msgs::PointStamped msg_pos) {
  static float currentPos_x = msg_pos.point.x;
  static float currentPos_y = msg_pos.point.y;
  static float hystereses = 1.2;

  // Shift hist_grid left, right, up, down TEST DIRECTIONS
  if (std::fabs(currentPos_x - msg_pos.point.x) > (hystereses*resolution_m/2))
  {
    int offsetX = floor((currentPos_x - msg_pos.point.x) / resolution_m);
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetX, 0, 1, 0);
    warpAffine(hist_grid,hist_grid,trans_mat,hist_grid.size());
    currentPos_x += offsetX; // CHECK sign
  }
  if (std::fabs(currentPos_y - msg_pos.point.y) > (hystereses*resolution_m/2))
  {
    int offsetY = floor((currentPos_y - msg_pos.point.y) / resolution_m);
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, offsetY);
    warpAffine(hist_grid,hist_grid,trans_mat,hist_grid.size());
    currentPos_y += offsetY; // CHECK sign
  }

  cv::Mat show;
  resize(hist_grid, show, cv::Size(), 10, 10, cv::INTER_NEAREST);
  imshow("asd", show);
  cv::waitKey(1);
}

void fillHistogramGrid(sensor_msgs::LaserScan msg_laser) {
  // Based on srcID, scalar times 90° is added to the yaw. CCW, Front = 0°
  std::string cameraID = msg_laser.header.frame_id;
  static int scalar; // this times 90° to rotate
  if (cameraID == "front"){scalar = 0;}
  else if (cameraID == "left"){scalar = 1;}
  else if (cameraID == "rear"){scalar = 2;}
  else if (cameraID == "right"){scalar = 3;}
  else {return;} // don't need down facing camera
  float yaw = rpy.vector.z;
  yaw *= scalar*M_PI/2;

  // Increment cell values at laserPoint with GRO mask and decrement cells along a line between center and laserPoint
  static int increment = 3;
  static int decrement = 1;
  cv::Point laserPoint;
  const int histCenter = (hist_grid.rows-1)/2;
  // GRO filter
  static cv::Mat_<float> kernel(3,3);
  kernel << 0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5;

  for (int i=0; i < msg_laser.ranges.size(); i++){
    if (msg_laser.ranges[i] != 0){
      laserPoint.x = histCenter + round(cos(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);
      laserPoint.y = histCenter + round(sin(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);

      // Increment
      if (hist_grid.at<unsigned char>(laserPoint) > 255 - increment){ //Overflow protection
        hist_grid.at<unsigned char>(laserPoint) = 255;
      }
      else{
        hist_grid.at<unsigned char>(laserPoint) += increment;
      }

      // Applying GRO mask
      cv::Mat onePixelSourceROI(hist_grid, cv::Rect( laserPoint, cv::Size(1, 1) ));
      cv::Mat dst (cv::Size(1,1), CV_8UC1);
      cv::filter2D(onePixelSourceROI, dst, CV_8UC1, kernel, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT); // TEST if borders are coppied
      hist_grid.at<unsigned char>(laserPoint) = dst.at<unsigned char>(0,0);

      // Decrement along a line
      cv::Mat linemask = cv::Mat::zeros(hist_grid.size(), CV_8UC1);
      line(linemask, cv::Point(histCenter, histCenter), laserPoint, cv::Scalar(255), 1, 8); // would 4 connectivity be better? TEST
      linemask.at<unsigned char>(laserPoint) = 0;
      cv::Mat histGridDec;
      hist_grid.copyTo(histGridDec);
      histGridDec -= decrement;
      histGridDec.copyTo(hist_grid, linemask);
    }
  }
  cv::Mat show;
  resize(hist_grid, show, cv::Size(), 10, 10, cv::INTER_NEAREST);
  imshow("asd", show);
  cv::waitKey(1);
}

void generate_LUTs() {
  const float b = 1;
  const float a = 1 + b*pow((hist_grid.rows-1)/2, 2);
  int x0 = floor(hist_grid.rows/2);
  int y0 = floor(hist_grid.cols/2);
  for(int i = 0; i < hist_grid.rows; i++) { //row
    for(int j = 0; j < hist_grid.cols; j++) { //column
      unsigned index = j+i*hist_grid.cols;
      beta[index]        = atan2(j-y0,i-x0) + C_PI;
      dist_scaled[index] = a - b * pow(sqrt(pow((x0-i),2) + pow((y0-j),2)),2);
      enlarge[index]     = asin(r_enl / sqrt(pow((x0-i),2) + pow((y0-j),2)));
    }
  }
}

void binary_hist() {
  float polar[s] = {0};

  for(int i = 0; i < hist_grid.rows; i++) {
    for(int j = 0; j < hist_grid.cols; j++) {
      unsigned index = j+i*hist_grid.cols;
      float mag = hist_grid.at<unsigned char>(j, i) * dist_scaled[index];
      for(int k = 0; k < s; k++) {
        if(k*alpha>=(beta[index]-enlarge[index]) && k*alpha<=(beta[index]+enlarge[index])){
          polar[k] += mag;
        }
      }
    }
  }

  for(int k = 0; k < s; k++) {
    if(polar[k] > t_high) {
      h[k] = 1;
    }
    else if(polar[k] < t_low) {
      h[k] = 0;
    }
    else {
      h[k] = prev_h[k];
    }
  }
  std::copy(std::begin(h), std::end(h), std::begin(prev_h));
}

void masked_polar_histogram() {
  float yaw = rpy.vector.z;
  float t_obst = 1;
  float max_rot_vel = 1;
  float r = velocity.vector.x/max_rot_vel; // Calculating the minimum steering radius, assumed that it´s the same for both directions
  double back = wrap_to_2pi(yaw-C_PI);
  back = RAD2DEG(back);
  yaw = RAD2DEG(yaw);

  // position of circle right (xr, yr) and left (xl, yl) relative to the drone position (middle of the hist_grid)
  float dxr = r * sin(yaw);
  float dyr = r * cos(yaw);
  float dxl = -dxr;
  float dyl = -dyr;

  float th_r = back;
  float th_l = back; // Setting the limit angles

  for (int i = 0; i < hist_grid.rows; i++)
  {
    for (int j=0; j < hist_grid.cols;j++)
    {
      unsigned index = j+i*hist_grid.cols;
      if (hist_grid.at<unsigned char>(j, i) > t_obst) {
        // If the hist_grid value is greter than cThreshold, enter loop
        if (checkright(yaw, beta[index], th_r) && blocked(dxr, dyr, i, j, r))
        {
          th_r = beta[index];
        }  // if angle is right to yaw and left to th_r: check blocked()
        else if (checkleft(yaw, beta[index], th_l) && blocked(dxl, dyl, i, j, r))
        {
          th_l = beta[index];
        }
      }
    }
  }

  // Building masked polar histogram
  for (int x=0; x < s; x++)
  {
    if (h[x] == 0 && inrange(x, th_l, th_r, yaw))
    {
      masked_hist[x] = 0;
    }
    else{
      masked_hist[x] = 1;
    }
  }
}

void get_borders() {
  k_l.clear();
  k_r.clear();

  unsigned current_val = masked_hist[0];
  for(unsigned i = 1; i < s; i++) {
    if(masked_hist[i] != current_val) {
      if(masked_hist[i] == 1) {
        k_r.push_back(i);
        current_val = 1;
      } else{
        k_l.push_back(i);
        current_val = 0;
      }
    }
  }

  if(k_r.size() != k_l.size() && masked_hist[0] == 0) {
    k_l.insert(k_l.begin(), 0);
  }
  if(k_r.size() != k_l.size() && masked_hist[0] == 1) {
    k_r.push_back(0);
  }
  if(masked_hist[0] == masked_hist[s-1] && masked_hist[0] == 0 && k_r.size() != 0) {
    k_r.push_back(k_r.front());
    k_r.erase(k_r.begin());
  }
}

void get_candidates() {
  c.clear();

  for(unsigned i = 0; i < k_l.size(); i++) {
    if(k_l.at(i) > k_r.at(i)) {
      unsigned valley_size = k_r.at(i) - k_l.at(i) + s;
      if(valley_size <= s_max) {
        double c_tmp = (k_r.at(i) + k_l.at(i) - s) / 2.0;
        if(c_tmp < 0) {
          c.push_back(c_tmp + s);
        } else{
          c.push_back(c_tmp);
        }
      } else {
        double c_tmp = k_r.at(i) - (s_max / 2.0);
        if(c_tmp < 0) {
          c.push_back(c_tmp + s);
        } else {
          c.push_back(c_tmp);
        }
        c_tmp = k_l.at(i) + (s_max / 2.0);
        if(c_tmp >= s) {
          c.push_back(c_tmp - s);
        } else {
          c.push_back(c_tmp);
        }
        if((k_target < k_r.at(i) && k_target + s > k_l.at(i)) || (k_target > k_l.at(i) && k_target < k_r.at(i) + s)) {
          c.push_back(k_target);
        }
      }
    } else {
      unsigned valley_size = k_r.at(i) - k_l.at(i);
      if(valley_size <= s_max) {
        c.push_back((k_r.at(i) + k_l.at(i)) / 2.0);
      } else {
        c.push_back(k_r.at(i) - (s_max / 2.0));
        c.push_back(k_l.at(i) + (s_max / 2.0));
        if(k_target > k_l.at(i) && k_target < k_r.at(i)) {
          c.push_back(k_target);
        }
      }
    }
  }
}

void calc_cost() {
  if(k_l.size() != 0 && k_r.size() != 0) {
    double g = 16384.0;
    for(auto &steering_dir : c) {
      double tmp_g = mu[0] * delta_c(steering_dir, k_target) +
      mu[1] * delta_c(steering_dir, RAD2DEG(rpy.vector.z)/alpha) +
      mu[2] * delta_c(steering_dir, prev_k_d);
      if(tmp_g < g) {
        k_d = steering_dir;
        g = tmp_g;
      }
    }
  } else {
    if(masked_hist[0] == 0) {
      k_d = k_target;
    } else {
      k_d = 0.0;
      // Handle this correctly. All directions are blocked.
    }
  }
  prev_k_d = k_d;
}

void publish_cmd() {
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp    = ros::Time::now();
  vel_cmd.header.frame_id = "vfh_vel_cmd";
  vel_cmd.twist.linear.x  = 1; // This needs to be correctly controlled
  vel_cmd.twist.angular.z = wrap_to_pi(DEG2RAD(k_d * alpha));
  vel_cmd_pub.publish(vel_cmd);
}

double delta_c(double c1, double c2) {
  return std::min(std::min(std::fabs(c1-c2-s), std::fabs(c1-c2+s)), std::fabs(c1-c2));
}

double wrap_to_pi(double angle) {
  angle = std::fmod(angle + C_PI, 2*C_PI);
  if(angle < 0) angle += 2 * C_PI;
  return angle - C_PI;
}

double wrap_to_2pi(double angle){
    angle = std::fmod(angle, 2*C_PI);
    if (angle < 0)
        angle += 2*C_PI;
    return angle;
}

bool blocked(float xt, float yt, float yc, float xc, float r) {   // xt,yt: coord of trajectory center, yc,xc: coordinates of active cell c(i,j)
  return (pow(std::fabs(xt-xc),2)+pow(std::fabs(yt-yc),2) < pow((r + r_enl),2)); // returns True if circles overlap
}

bool checkright(float y, float b, float r){
if(y < 180) {
  y += 180;
  b = std::fmod((b+180),360);
  r = std::fmod((r+180),360);
}
return ((y > b) && (b > r));
}

bool checkleft(float y, float b, float l){
if (y > 180) {
  y += 180;
  b = std::fmod((b+180),360);
  l = std::fmod((l+180),360);
}
return ((y < b) && (b < l));
}

bool inrange(int x, float th_l, float th_r, float yaw){
  float di = alpha*x + alpha/2;
  return (checkleft(yaw, di, th_l) || checkright(yaw, di, th_r));
}
