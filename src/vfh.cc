#include "uav_nav/vfh.h"

// Global variables
ros::ServiceClient            vfh_luts;       // Service client
ros::Publisher                vel_cmd_pub;    // Linear x velocity, and yaw rate
geometry_msgs::PointStamped   local_position; // Local position offset in FLU frame
geometry_msgs::Vector3Stamped rpy;            // Roll, pitch, yaw
geometry_msgs::Vector3Stamped velocity;       // Linear velocity
cv::Mat                       hist_grid;      // Histogram grid 2D

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "vfh");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh_("~");

  // Local variables
  int                   s;                             // Number of angular sectors
  float                 bin_hist_high;                 // Binary histogram high threshold
  float                 bin_hist_low;                  // Binary histogram low threshold
  float                 radius_enlargement;            // Robot + safety radius
  std::vector<float>    cost_params;                   // Three cost parameters
  std::vector<float>    target_xy;                     // Target for the drone
  static const float    cost_default[]      = {5,2,2}; // Default cost parameters
  static const float    target_default[]    = {0,0};   // Default target [x, y]
  std::vector<float>    beta;                          // Histogram grid cell angle from RCP
  std::vector<float>    dist_scaled;                   // Histogram grid cell distance from RCP
  std::vector<float>    enlarge;                       // Obstacle enlargement angle from RCP
  std::vector<unsigned> h;                             // Binary polar histogram
  std::vector<unsigned> masked_hist;                   // Masked binary polar histogram
  std::vector<int>      k_l;                           // Right borders of candidate valleys
  std::vector<int>      k_r;                           // Left borders of candidate valleys
  std::vector<float>    c;                             // Candidate directions
  float                 k_d                 = 0.0;     // Selected direction of motion
  unsigned              alpha               = 360 / s; // Sector angle
  float                 k_target            = 0.0;     // Target direction

  // Histogram grid setup
  static int histDimension = round((float)CAMERARANGE*2/RESOLUTION_M);
  if(histDimension%2 != 1) histDimension++;
  hist_grid = cv::Mat::zeros(histDimension, histDimension, CV_8UC1);

  // Load parameters
  private_nh_.param("s",           s,                  72);
  private_nh_.param("t_high",      bin_hist_high,      1.f);
  private_nh_.param("t_low",       bin_hist_low,       1.f);
  private_nh_.param("r_enl",       radius_enlargement, 1.f);
  private_nh_.param("cost_params", cost_params,        std::vector<float>(cost_default,   cost_default+3));
  private_nh_.param("target",      target_xy,          std::vector<float>(target_default, target_default+2));

  //Services
  vfh_luts = nh.serviceClient<uav_nav::VFHLookUpTables>("uav_nav/vfh_luts");

  //Subsriber
  ros::Subscriber loc_pos_sub    = nh.subscribe("dji_sdk/local_position",           1, &LocalPositionCb);
  ros::Subscriber vel_sub        = nh.subscribe("dji_sdk/velocity",                 1, &VelocityCb);
  ros::Subscriber rpy_sub        = nh.subscribe("uav_nav/roll_pitch_yaw",           1, &RollPitchYawCb);
  ros::Subscriber laser_scan_sub = nh.subscribe("uav_nav/laser_scan_from_depthIMG", 1, &LaserScanCb);

  //Publishers
  vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("uav_nav/vel_cmd", 1);

  // Necessary functions before entering ros spin
  GetLUTs(histDimension, radius_enlargement, beta, dist_scaled, enlarge);
  //ros::Rate r(1); //1Hz

  while(ros::ok())
  {
    GetTargetDir(alpha, target_xy, k_target);
    BinaryHist(s, alpha, bin_hist_high, bin_hist_low, beta, dist_scaled, enlarge, h);
    MaskedPolarHist(alpha, radius_enlargement, beta, h, masked_hist);
    FindValleyBorders(masked_hist, k_l, k_r);
    FindCandidateDirections(s, k_target, k_l, k_r, c);
    CalculateCost(s, alpha, k_target, c, cost_params, masked_hist, k_d);
    PublishCtrlCmd(k_d, alpha);

    ros::spinOnce();
    //r.sleep();
  }

  return 0;
}

// Callbacks
void LocalPositionCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position = *msg;
  ShiftHistogramGrid(local_position);
}

void VelocityCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  velocity = *msg;
}

void RollPitchYawCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  rpy = *msg;
}

void LaserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  FillHistogramGrid(*msg);
}

// Functions
void GetTargetDir(unsigned           alpha,
                  std::vector<float> target,
                  float             &k_target
                 )
{
  float target_angle = std::atan2(target[1]-local_position.point.y, target[0]-local_position.point.x);
  float angle_diff = WrapToPi(target_angle-rpy.vector.z);
  k_target = RAD2DEG(angle_diff)/alpha;
}

void GetLUTs(int                size,
             float              radius,
             std::vector<float> &beta,
             std::vector<float> &dist_scaled,
             std::vector<float> &enlarge
            )
{
  vfh_luts.waitForExistence();

  uav_nav::VFHLookUpTables lut;
  lut.request.size = size;
  lut.request.radius = radius;
  vfh_luts.call(lut);

  beta        = lut.response.beta;
  dist_scaled = lut.response.dist;
  enlarge     = lut.response.gamma;
}

void FillHistogramGrid(sensor_msgs::LaserScan msg_laser)
{
  // Based on srcID, scalar times 90° is added to the yaw. CCW, Front = 0°
  std::string cameraID = msg_laser.header.frame_id;
  static int scalar; // this times 90° to rotate
  if(cameraID == "front"){scalar = 0;}
  else if(cameraID == "left"){scalar = 1;}
  else if(cameraID == "rear"){scalar = 2;}
  else if(cameraID == "right"){scalar = 3;}
  else {return;} // don't need down facing camera
  //float yaw = rpy.vector.z;
  float yaw = 0;
  yaw += scalar*M_PI/2;

  // Increment cell values at laserPoint with GRO mask and decrement cells along a line between center and laserPoint
  static int increment = 3;
  static int decrement = 1;
  cv::Point laserPoint;
  const int histCenter = (hist_grid.rows-1)/2;
  // GRO filter
  static cv::Mat_<float> kernel(3,3);
  kernel << 0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5;

  for(int i=0; i < msg_laser.ranges.size(); i++)
  {
    if(msg_laser.ranges[i] != 0)
    {
      laserPoint.x = histCenter + round(cos(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);
      laserPoint.y = histCenter + round(sin(yaw + msg_laser.angle_max - i * msg_laser.angle_increment) * msg_laser.ranges[i]);

      // Increment
      if(hist_grid.at<unsigned char>(laserPoint) > 255 - increment)
      { //Overflow protection
        hist_grid.at<unsigned char>(laserPoint) = 255;
      }
      else
      {
        hist_grid.at<unsigned char>(laserPoint) += increment;
      }

      // Applying GRO mask
      cv::Mat onePixelSourceROI(hist_grid, cv::Rect(laserPoint, cv::Size(1, 1)));
      cv::Mat dst(cv::Size(1,1), CV_8UC1);
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

void ShiftHistogramGrid(geometry_msgs::PointStamped msg_pos)
{
  static float currentPos_x = msg_pos.point.x;
  static float currentPos_y = msg_pos.point.y;
  static float hystereses = 1.2;

  // Shift grid left, right, up, down TEST DIRECTIONS
  if(std::fabs(currentPos_x - msg_pos.point.x) > (hystereses*RESOLUTION_M/2))
  {
    int offsetX = floor((currentPos_x - msg_pos.point.x) / RESOLUTION_M);
    cv::Mat trans_mat = (cv::Mat_<float>(2,3) << 1, 0, offsetX, 0, 1, 0);
    warpAffine(hist_grid,hist_grid,trans_mat,hist_grid.size());
    currentPos_x += offsetX; // CHECK sign
  }

  if(std::fabs(currentPos_y - msg_pos.point.y) > (hystereses*RESOLUTION_M/2))
  {
    int offsetY = floor((currentPos_y - msg_pos.point.y) / RESOLUTION_M);
    cv::Mat trans_mat = (cv::Mat_<float>(2,3) << 1, 0, 0, 0, 1, offsetY);
    warpAffine(hist_grid,hist_grid,trans_mat,hist_grid.size());
    currentPos_y += offsetY; // CHECK sign
  }

  cv::Mat show;
  resize(hist_grid, show, cv::Size(), 10, 10, cv::INTER_NEAREST);
  imshow("asd", show);
  cv::waitKey(1);
}

void BinaryHist(unsigned              s,
                unsigned              alpha,
                float                 t_high,
                float                 t_low,
                std::vector<float>    beta,
                std::vector<float>    dist_scaled,
                std::vector<float>    enlarge,
                std::vector<unsigned> &h
               )
{
  float polar[s] = {0};

  for(int i = 0; i < hist_grid.rows; i++)
  {
    for(int j = 0; j < hist_grid.cols; j++)
    {
      unsigned index = j+i*hist_grid.cols;
      float mag = hist_grid.at<unsigned char>(j, i) * dist_scaled[index];
      for(int k = 0; k < s; k++)
      {
        if(k*alpha>=(beta[index]-enlarge[index]) && k*alpha<=(beta[index]+enlarge[index]))
          polar[k] += mag;
      }
    }
  }

  static std::vector<unsigned> prev_h(s); // check if this works correctly
  h.clear();
  for(int k = 0; k < s; k++)
  {
    if(polar[k] > t_high)
    {
      h.push_back(1);
    }
    else if(polar[k] < t_low)
    {
      h.push_back(0);
    }
    else
    {
      h.push_back(prev_h[k]);
    }
  }

  prev_h = h;
}

void MaskedPolarHist(unsigned              alpha,
                     float                 r_enl,
                     std::vector<float>    beta,
                     std::vector<unsigned> h,
                     std::vector<unsigned> &masked_hist
                    )
{
  float yaw = rpy.vector.z;
  float t_obst = 1;
  float max_rot_vel = 1;
  float r = velocity.vector.x/max_rot_vel; // Calculating the minimum steering radius, assumed that it´s the same for both directions
  float back = WrapTo2Pi(yaw-C_PI);
  back = RAD2DEG(back);
  yaw = RAD2DEG(yaw);

  // position of circle right (xr, yr) and left (xl, yl) relative to the drone position (middle of the grid)
  float dxr = r * sin(yaw);
  float dyr = r * cos(yaw);
  float dxl = -dxr;
  float dyl = -dyr;

  float th_r = back;
  float th_l = back; // Setting the limit angles

  for(int i = 0; i < hist_grid.rows; i++)
  {
    for(int j=0; j < hist_grid.cols;j++)
    {
      unsigned index = j+i*hist_grid.cols;
      if(hist_grid.at<unsigned char>(j, i) > t_obst)
      {
        // If the grid value is greter than cThreshold, enter loop
        if(CheckRight(yaw, beta[index], th_r) && Blocked(dxr, dyr, i, j, r+r_enl))
        {
          th_r = beta[index];
        }  // if angle is right to yaw and left to th_r: check Blocked()
        else if(CheckLeft(yaw, beta[index], th_l) && Blocked(dxl, dyl, i, j, r+r_enl))
        {
          th_l = beta[index];
        }
      }
    }
  }

  // Building masked polar histogram
  masked_hist.clear();
  for(int x=0; x < h.size(); x++)
  {
    if(h[x] == 0 && InRange(alpha, x, th_l, th_r, yaw))
    {
      masked_hist.push_back(0);
    }
    else
    {
      masked_hist.push_back(1);
    }
  }
}

void FindValleyBorders(std::vector<unsigned> masked_hist,
                       std::vector<int>      &k_l,
                       std::vector<int>      &k_r
                      )
{
  k_l.clear();
  k_r.clear();

  unsigned current_val = masked_hist[0];
  for(unsigned i = 1; i < masked_hist.size(); i++)
  {
    if(masked_hist[i] != current_val)
    {
      if(masked_hist[i] == 1)
      {
        k_r.push_back(i);
        current_val = 1;
      }
      else
      {
        k_l.push_back(i);
        current_val = 0;
      }
    }
  }

  if(k_r.size() != k_l.size() && masked_hist[0] == 0)
    k_l.insert(k_l.begin(), 0);

  if(k_r.size() != k_l.size() && masked_hist[0] == 1)
    k_r.push_back(0);

  if(masked_hist[0] == masked_hist[masked_hist.size()-1] && masked_hist[0] == 0 && k_r.size() != 0)
  {
    k_r.push_back(k_r.front());
    k_r.erase(k_r.begin());
  }
}

void FindCandidateDirections(unsigned           s,
                             float              k_target,
                             std::vector<int>   k_l,
                             std::vector<int>   k_r,
                             std::vector<float> &c
                            )
{
  c.clear();
  static const unsigned s_max = 16; // Min number of sectors for a wide valley

  for(unsigned i = 0; i < k_l.size(); i++)
  {
    if(k_l.at(i) > k_r.at(i))
    {
      unsigned valley_size = k_r.at(i) - k_l.at(i) + s;
      if(valley_size <= s_max)
      {
        float c_tmp = (k_r.at(i) + k_l.at(i) - s) / 2.0;
        if(c_tmp < 0)
        {
          c.push_back(c_tmp + s);
        }
        else
        {
          c.push_back(c_tmp);
        }
      }
      else
      {
        float c_tmp = k_r.at(i) - (s_max / 2.0);
        if(c_tmp < 0)
        {
          c.push_back(c_tmp + s);
        }
        else
        {
          c.push_back(c_tmp);
        }
        c_tmp = k_l.at(i) + (s_max / 2.0);
        if(c_tmp >= s)
        {
          c.push_back(c_tmp - s);
        }
        else
        {
          c.push_back(c_tmp);
        }
        if((k_target < k_r.at(i) && k_target + s > k_l.at(i)) || (k_target > k_l.at(i) && k_target < k_r.at(i) + s))
        {
          c.push_back(k_target);
        }
      }
    }
    else
    {
      unsigned valley_size = k_r.at(i) - k_l.at(i);
      if(valley_size <= s_max)
      {
        c.push_back((k_r.at(i) + k_l.at(i)) / 2.0);
      }
      else
      {
        c.push_back(k_r.at(i) - (s_max / 2.0));
        c.push_back(k_l.at(i) + (s_max / 2.0));
        if(k_target > k_l.at(i) && k_target < k_r.at(i))
          c.push_back(k_target);
      }
    }
  }
}

void CalculateCost(unsigned              s,
                   unsigned              alpha,
                   float                 k_target,
                   std::vector<float>    c,
                   std::vector<float>    mu,
                   std::vector<unsigned> masked_hist,
                   float                 &k_d
                  )
{
  static float prev_k_d = 0.0; // Previous direction of motion

  if(c.size() != 0)
  {
    float g = 16384.0;
    for(auto &steering_dir : c)
    {
      float tmp_g = mu[0] * DeltaC(steering_dir, k_target,                    s) +
                    mu[1] * DeltaC(steering_dir, RAD2DEG(rpy.vector.z)/alpha, s) +
                    mu[2] * DeltaC(steering_dir, prev_k_d,                    s);
      if(tmp_g < g)
      {
        k_d = steering_dir;
        g = tmp_g;
      }
    }
  }
  else
  {
    if(masked_hist[0] == 0)
    {
      k_d = k_target;
    }
    else
    {
      k_d = 0.0;
      // Handle this correctly. All directions are Blocked.
    }
  }

  prev_k_d = k_d;
}

void PublishCtrlCmd(float    k_d,
                    unsigned alpha
                   )
{
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp    = ros::Time::now();
  vel_cmd.header.frame_id = "vfh_vel_cmd";
  vel_cmd.twist.linear.x  = 1; // This needs to be correctly controlled
  vel_cmd.twist.angular.z = WrapToPi(DEG2RAD(k_d * alpha));
  vel_cmd_pub.publish(vel_cmd);
}

float DeltaC(float    c1,
             float    c2,
             unsigned s
            )
{
  return std::min(std::min(std::fabs(c1-c2-s), std::fabs(c1-c2+s)), std::fabs(c1-c2));
}

float WrapToPi(float angle) {
  angle = std::fmod(angle + C_PI, 2*C_PI);
  if(angle < 0) angle += 2 * C_PI;
  return (angle - C_PI);
}

float WrapTo2Pi(float angle)
{
  angle = std::fmod(angle, 2*C_PI);
  if(angle < 0)
    angle += 2*C_PI;
  return angle;
}

bool Blocked(float xt, // x coordinate of trajectory center
             float yt, // y coordinate of trajectory center
             float yc, // y coordinate of active cell
             float xc, // x coordinate of active cell
             float r   // Turning + safety radius
            )
{
  return (pow(xt-xc,2)+pow(yt-yc,2) < pow(r,2)); // returns True if circles overlap
}

bool CheckRight(float y,
                float b,
                float r
               )
{
  if(y < 180)
  {
    y += 180;
    b = std::fmod((b+180),360);
    r = std::fmod((r+180),360);
  }

  return ((y > b) && (b > r));
}

bool CheckLeft(float y,
               float b,
               float l
              )
{
  if (y > 180)
  {
    y += 180;
    b = std::fmod((b+180),360);
    l = std::fmod((l+180),360);
  }

  return ((y < b) && (b < l));
}

bool InRange(unsigned alpha,
             int      x,
             float    th_l,
             float    th_r,
             float    yaw
            )
{
  float di = alpha*x + alpha/2;
  return (CheckLeft(yaw, di, th_l) || CheckRight(yaw, di, th_r));
}
