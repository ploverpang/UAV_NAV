#include "uav_nav/vfh.h"
#include <geometry_msgs/TwistStamped.h>

// Global variables
ros::ServiceClient            vfh_luts;       // Service client
geometry_msgs::PointStamped   local_position; // Local position offset in FLU frame
geometry_msgs::Vector3Stamped rpy;            // Roll, pitch, yaw
geometry_msgs::Vector3Stamped velocity;       // Linear velocity
cv::Mat                       hist_grid;      // Histogram grid 2D
cv::Mat                       circle_mask;    // Mask used to create circular active window
float                 k_target            = 0.0;     // Target direction
std::string id_buffer = "zero";
int targetReached = 0;
int upscalar = 10;
cv::Mat show;
int run = 0;


void vfh_start(const std_msgs::Bool::ConstPtr& msg){
  if (msg->data == true) run = 1;
}



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "vfh_plot");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh_("~");

  // Parameter variables
  int                   s;                             // Number of angular sectors
  float                 bin_hist_high;                 // Binary histogram high threshold
  float                 bin_hist_low;                  // Binary histogram low threshold
  float                 radius_enlargement;            // Robot + safety radius
  std::vector<float>    cost_params;                   // Three cost parameters
  std::vector<float>    target_xy;                     // Target for the drone
  static const float    cost_default[]      = {5,2,2}; // Default cost parameters
  static const float    target_default[]    = {0,0};   // Default target [x, y]
  float                 t_obst;                        // Obstacle threshold


  // Load parameters
  private_nh_.param("/vfh/t_obst",      t_obst,             60.f);
  private_nh_.param("/vfh/s",           s,                  72);
  private_nh_.param("/vfh/t_obst",      t_obst,             60.f);
  private_nh_.param("/vfh/t_high",      bin_hist_high,      20.5f);
  private_nh_.param("/vfh/t_low",       bin_hist_low,       15.f);
  private_nh_.param("/vfh/r_enl",       radius_enlargement, 1.f);
  private_nh_.param("/vfh/cost_params", cost_params,        std::vector<float>(cost_default,   cost_default+3));
  private_nh_.param("/vfh/target",      target_xy,          std::vector<float>(target_default, target_default+2));

  // Local variables
  std::vector<float>    beta;                          // Histogram grid cell angle from RCP
  std::vector<float>    dist_scaled;                   // Histogram grid cell distance from RCP
  std::vector<float>    enlarge;                       // Obstacle enlargement angle from RCP
  std::vector<unsigned> h;                             // Binary polar histogram
  std::vector<unsigned> masked_hist;                   // Masked binary polar histogram
  std::vector<int>      k_l;                           // Right borders of candidate valleys
  std::vector<int>      k_r;                           // Left borders of candidate valleys
  std::vector<float>    c;                             // Candidate directions
  //float                 k_target            = 0.0;     // Target direction
  float                 k_d                 = 0.0;     // Selected direction of motion
  float                 lin_vel             = 0.0;     //
  unsigned              vel_flag            = 0;
  unsigned              alpha               = 360 / s; // Sector angle
  bool                  red_vel             = false;   // Flag inicating reduction needed in linear velocity


  // Histogram grid setup
  static int histDimension = round((float)CAMERARANGE*1.5*2/RESOLUTION_M);
  if(histDimension%2 != 1)
    histDimension++;
  hist_grid = cv::Mat::zeros(histDimension, histDimension, CV_8UC1);
  circle_mask = cv::Mat::zeros(histDimension, histDimension, CV_8UC1);
  circle(circle_mask, cv::Point((hist_grid.rows-1)/2, (hist_grid.cols-1)/2), static_cast<int>(CAMERARANGE*1.5)*2, cv::Scalar(255), -1, 8, 0);


  //Services
  vfh_luts = nh.serviceClient<uav_nav::VFHLookUpTables>("uav_nav/vfh_luts");

  //Subsriber
  ros::Subscriber loc_pos_sub    = nh.subscribe("dji_sdk/local_position",           1, &localPositionCb);
  ros::Subscriber vel_sub        = nh.subscribe("dji_sdk/velocity",                 1, &velocityCb);
  ros::Subscriber rpy_sub        = nh.subscribe("uav_nav/roll_pitch_yaw",           1, &RPYCb);
  ros::Subscriber laser_scan_sub = nh.subscribe("uav_nav/laser_scan_from_depthIMG_debug", 3, &laserScanCb);
  ros::Subscriber vfh_started    = nh.subscribe("uav_nav/vel_cmd", 1, &vfh_start);
  ros::Subscriber ready_sub = nh.subscribe("uav_nav/dc_ready", 1, &vfh_start);

  // Necessary functions before entering ros spin
  getLUTs(histDimension, radius_enlargement, &beta, &dist_scaled, &enlarge);

  std::vector<float> FLUtarget;
  FLUtarget.push_back(target_xy[0]*cos(rpy.vector.z+C_PI/2)-target_xy[1]*sin(rpy.vector.z+C_PI/2));
  FLUtarget.push_back(target_xy[0]*sin(rpy.vector.z+C_PI/2)+target_xy[1]*cos(rpy.vector.z+C_PI/2));

  while(ros::ok())
  {
    private_nh_.getParam("/vfh/target", target_xy);

    if(run){
    resize(hist_grid, show, cv::Size(), upscalar, upscalar, cv::INTER_NEAREST);
    cvtColor(show, show, CV_GRAY2RGB);

    getTargetDir(alpha, FLUtarget, &k_target);
    binaryHist(s, alpha, bin_hist_high, bin_hist_low, beta, dist_scaled, enlarge, &h);
    maskedPolarHist(alpha, radius_enlargement, t_obst, beta, h, &red_vel, &masked_hist);
    findValleyBorders(masked_hist, &k_l, &k_r);
    findCandidateDirections(s, k_target, k_l, k_r, &c);
    calculateCost(s, alpha, k_target, c, cost_params, masked_hist, &k_d);

    // Debug only
    imshow("Histogram grid", show);
    cv::waitKey(1);
    }
    ros::spinOnce();
  }

  return 0;
}

// Callbacks
void localPositionCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position = *msg;
  shiftHistogramGrid();
}

void velocityCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  velocity = *msg;
}

void RPYCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  rpy = *msg;
}

void laserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  fillHistogramGrid(*msg);
}

// Functions
void getLUTs(int                size,
             float              radius,
             std::vector<float> *beta,
             std::vector<float> *dist_scaled,
             std::vector<float> *enlarge
            )
{
  vfh_luts.waitForExistence();

  uav_nav::VFHLookUpTables lut;
  lut.request.size   = size;
  lut.request.radius = radius;
  vfh_luts.call(lut);

  *beta        = lut.response.beta;
  *dist_scaled = lut.response.dist;
  *enlarge     = lut.response.gamma;
}

void getTargetDir(unsigned                 alpha,
                  const std::vector<float> &target,
                  float                    *k_target
                 )
{
  float target_angle = wrapToPi(std::atan2(target[1]-local_position.point.y, target[0]-local_position.point.x) - C_PI/2);
  float angle_diff = wrapTo2Pi(target_angle-rpy.vector.z);
  *k_target = RAD2DEG(angle_diff)/alpha;
}

void fillHistogramGrid(sensor_msgs::LaserScan msg)
{
  // Based on camera_ID, scalar * 90° is added to the yaw. CCW, north = 0°

  if (id_buffer == msg.header.frame_id) return;
  id_buffer = msg.header.frame_id;
  if(!run)return;

  static int scalar;
  std::string camera_ID = msg.header.frame_id;

  if(camera_ID == "front")
    scalar = 0;
  else if(camera_ID == "left")
    scalar = 1;
  else if(camera_ID == "rear")
    scalar = 2;
  else if(camera_ID == "right")
    scalar = 3;
  else
    return;

  float yaw = rpy.vector.z + scalar*C_PI/2;

  // Increment cell values at laser_point with GRO mask and decrement cells along a line between center and laser_point
  static const int increment = 3;
  static const int decrement = 1;
  static const int hist_center = (hist_grid.rows-1)/2;
  static const cv::Mat kernel = (cv::Mat_<float>(3,3) << 0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5); // GRO filter
  static cv::Point laser_point;

  for(int i = 0; i < msg.ranges.size(); ++i)
  {
    if(msg.ranges[i] > 0.5)
    {
      laser_point.x = hist_center - round(sin(yaw + msg.angle_max - i * msg.angle_increment) * msg.ranges[i] / RESOLUTION_M);
      laser_point.y = hist_center - round(cos(yaw + msg.angle_max - i * msg.angle_increment) * msg.ranges[i] / RESOLUTION_M);

      // Increment
      if(hist_grid.at<unsigned char>(laser_point) > 255 - increment)
        hist_grid.at<unsigned char>(laser_point) = 255; // Overflow protection
      else
        hist_grid.at<unsigned char>(laser_point) += increment;

      // Applying GRO mask
      cv::Mat one_pixel_ROI(hist_grid, cv::Rect(laser_point, cv::Size(1, 1)));
      cv::Mat dst(cv::Size(1, 1), CV_8UC1);
      cv::filter2D(one_pixel_ROI, dst, CV_8UC1, kernel, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);
      hist_grid.at<unsigned char>(laser_point) = dst.at<unsigned char>(0,0);

      // Decrement along a line
      cv::Mat linemask = cv::Mat::zeros(hist_grid.size(), CV_8UC1);
      line(linemask, cv::Point(hist_center, hist_center), laser_point, cv::Scalar(255), 1, 4);
      linemask.at<unsigned char>(laser_point) = 0;
      cv::Mat hist_grid_dec;
      hist_grid.copyTo(hist_grid_dec);
      hist_grid_dec -= decrement;
      hist_grid_dec.copyTo(hist_grid, linemask);
    }
  }
}

void shiftHistogramGrid()
{
  if(!run)return;
  static float current_pos_x = local_position.point.x;
  static float current_pos_y = local_position.point.y;
  static const float hysteresis = 1.2;

  // Shift grid left, right, up, down
  float displacement_x = local_position.point.x - current_pos_x;
  if(std::fabs(displacement_x) > (hysteresis*RESOLUTION_M/2))
  {
    int offset_x = -trunc(displacement_x / RESOLUTION_M);
    cv::Mat trans_mat = (cv::Mat_<float>(2,3) << 1, 0, offset_x, 0, 1, 0);
    /*#ifdef USE_GPU
    cv::cuda::GpuMat hist_grid_cuda, trans_mat_cuda;
    trans_mat_cuda.upload(trans_mat);
    hist_grid_cuda.upload(hist_grid);
    cv::cuda::warpAffine(hist_grid, hist_grid, trans_mat, hist_grid.size());
    hist_grid_cuda.download(hist_grid);
    #else*/
    warpAffine(hist_grid, hist_grid, trans_mat, hist_grid.size());
    //#endif
    current_pos_x = current_pos_x + std::copysign((RESOLUTION_M*offset_x), displacement_x);
    cv::Mat masked;
    hist_grid.copyTo(masked, circle_mask);
    hist_grid = masked;
  }

  float displacement_y = local_position.point.y - current_pos_y;
  if(std::fabs(displacement_y) > (hysteresis*RESOLUTION_M/2))
  {
    int offset_y = trunc(displacement_y / RESOLUTION_M);
    cv::Mat trans_mat = (cv::Mat_<float>(2,3) << 1, 0, 0, 0, 1, offset_y);
    /*#ifdef USE_GPU
    cv::cuda::GpuMat hist_grid_cuda, trans_mat_cuda;
    trans_mat_cuda.upload(trans_mat);
    hist_grid_cuda.upload(hist_grid);
    cv::cuda::warpAffine(hist_grid, hist_grid, trans_mat, hist_grid.size());
    hist_grid_cuda.download(hist_grid);
    #else*/
    warpAffine(hist_grid, hist_grid, trans_mat, hist_grid.size());
    //#endif
    current_pos_y = current_pos_y + std::copysign((RESOLUTION_M*offset_y), displacement_y);
    cv::Mat masked;
    hist_grid.copyTo(masked, circle_mask);
    hist_grid = masked;
  }
}

void binaryHist(unsigned                 s,
                unsigned                 alpha,
                float                    t_high,
                float                    t_low,
                const std::vector<float> &beta,
                const std::vector<float> &dist_scaled,
                const std::vector<float> &enlarge,
                std::vector<unsigned>    *h
               )
{
  float polar[72] = {0};
  for(int i = 0; i < hist_grid.rows; ++i)
  {
    for(int j = 0; j < hist_grid.cols; ++j)
    {
      unsigned index = j+(i*hist_grid.cols);
      float magnitude = pow((float)hist_grid.at<unsigned char>(i, j)/255.0, 2) * dist_scaled[index];
      if(magnitude > 0)
      {
        for(int k = 0; k < s; ++k)
        {
          if(isBetweenRad(wrapTo2Pi(beta[index])-enlarge[index], wrapTo2Pi(beta[index])+enlarge[index], k*DEG2RAD(alpha)))
            polar[k] += magnitude;
        }
      }
    }
  }
  /*ROS_INFO("\nPolar histogram values in this iteration are:");
  for (int i = 0; i < s; ++i)
  {
    if(polar[i]!=0){ROS_INFO("Sector %i has value %f", i, polar[i]);}
  }*/

  static std::vector<unsigned> prev_h(s);
  h->clear(); // Same as (*h).clear();
  for(int k = 0; k < s; ++k)
  {
    if(polar[k] > t_high)
      h->push_back(1);
    else if(polar[k] < t_low)
      h->push_back(0);
    else
      h->push_back(prev_h[k]);
  }

  prev_h = *h;
}

void maskedPolarHist(unsigned                    alpha,
                     float                       r_enl,
                     float                       t_obst,
                     const std::vector<float>    &beta,
                     const std::vector<unsigned> &h,
                     bool                        *red_vel,
                     std::vector<unsigned>       *masked_hist
                    )
{
  float yaw  = rpy.vector.z;                  // Heading of drone in radians
  float r    = sqrt(
                pow(velocity.vector.x,2) +
                pow(velocity.vector.y,2)
                   ) / MAXROTVEL;             // Minimum steering radius assuming it is the same for both directions
  float back = wrapToPi(yaw-C_PI);            // Opposite direction of heading
  float dxr  = r * sin(yaw);                  // X coord. of right trajectory circle
  float dyr  = r * cos(yaw);                  // Y coord. of right trajectory circle
  float dxl  = -dxr;                          // X coord. of left trajectory circle
  float dyl  = -dyr;                          // Y coord. of left trajectory circle
  float th_r = back;                          // Right limit angle
  float th_l = back;                          // Left limit angle

  *red_vel = false;

  for(int i = 0; i < hist_grid.rows; ++i)
  {
    for(int j = 0; j < hist_grid.cols; ++j)
    {
      unsigned index = j+(i*hist_grid.cols);
      if(hist_grid.at<unsigned char>(i, j) > t_obst)
      {
        if(isBetweenRad(th_r, yaw, beta[index]) && blocked(dxr, dyr, j, i, r+r_enl))
          th_r = beta[index];
        else if(isBetweenRad(yaw, th_l, beta[index]) && blocked(dxl, dyl, j, i, r+r_enl))
          th_l = beta[index];
      }
    }
  }

  for(int i = 0; i < show.rows; ++i)
  {
    for(int j = 0; j < show.cols; ++j)
    {
      cv::Vec3b Intens = show.at<cv::Vec3b>(i, j);
      if(Intens.val[0] > t_obst)
      {
        show.at<cv::Vec3b>(i, j) = cv::Vec3b(255,0,0);
      }
    }
  }

  cv::Mat sector_block;
  show.copyTo(sector_block);
  sector_block = cv::Vec3b(0,0,0);
  double alpha_blend = 0.3;

  // Create masked polar histogram
  bool all_blocked = true;
  masked_hist->clear();
  for(int k = 0; k < h.size(); ++k)
  {
    if(h[k] == 0 && (isBetweenRad(th_r, yaw, DEG2RAD(alpha)*k) || isBetweenRad(yaw, th_l, DEG2RAD(alpha)*k))){
      masked_hist->push_back(0);
      all_blocked = false;
    }
    else{
      masked_hist->push_back(1);
      int hist_center = (show.rows-1)/2;
      int radius_circle = round((float)CAMERARANGE*1.5/RESOLUTION_M)*upscalar;
      cv::Point p0 = cv::Point(0, radius_circle);
      cv::Point center_point = cv::Point(hist_center, hist_center);
      cv::Point poly[1][4];
      poly[0][0] = center_point;
      poly[0][1] = cv::Point(p0.x*cos(k*DEG2RAD(alpha))-p0.y*sin(k*DEG2RAD(alpha)) + (show.rows-1)/2, -(p0.x*sin((k)*DEG2RAD(alpha))+p0.y*cos((k)*DEG2RAD(alpha))) +(show.rows-1)/2 );
      poly[0][2] = cv::Point(p0.x*cos((k+1)*DEG2RAD(alpha))-p0.y*sin((k+1)*DEG2RAD(alpha)) + (show.rows-1)/2, -(p0.x*sin((k+1)*DEG2RAD(alpha))+p0.y*cos((k+1)*DEG2RAD(alpha))) +(show.rows-1)/2 );
      poly[0][3] = center_point;
      const cv::Point* ppt[1] = {poly[0]};
      int npt[] = {4};
      cv::fillPoly(sector_block, ppt, npt, 1, cv::Scalar(0, 0, 255), 8);
    }
  }
    cv::addWeighted(show, 1, sector_block, alpha_blend/2, 0, show);


  if(all_blocked)
  {
    *red_vel = true;

    r *= 0.5;
    float th_r = back;                          // Right limit angle
    float th_l = back;                          // Left limit angle

    for(int i = 0; i < hist_grid.rows; ++i)
    {
      for(int j = 0; j < hist_grid.cols; ++j)
      {
        unsigned index = j+(i*hist_grid.cols);
        if(hist_grid.at<unsigned char>(i, j) > t_obst)
        {
          if(isBetweenRad(th_r, yaw, beta[index]) && blocked(dxr, dyr, j, i, r+r_enl))
            th_r = beta[index];
          else if(isBetweenRad(yaw, th_l, beta[index]) && blocked(dxl, dyl, j, i, r+r_enl))
            th_l = beta[index];
        }
      }
    }

    // Create masked polar histogram
    masked_hist->clear();
    for(int k = 0; k < h.size(); ++k)
    {
      if(h[k] == 0 && (isBetweenRad(th_r, yaw, DEG2RAD(alpha)*k) || isBetweenRad(yaw, th_l, DEG2RAD(alpha)*k))){
        masked_hist->push_back(0);
      }
      else{
        masked_hist->push_back(1);
        int hist_center = (show.rows-1)/2;
      int radius_circle = round((float)CAMERARANGE*1.5/RESOLUTION_M)*upscalar;
      cv::Point p0 = cv::Point(0, radius_circle);
      cv::Point center_point = cv::Point(hist_center, hist_center);
      cv::Point poly[1][4];
      poly[0][0] = center_point;
      poly[0][1] = cv::Point(p0.x*cos(k*DEG2RAD(alpha))-p0.y*sin(k*DEG2RAD(alpha)) + (show.rows-1)/2, -(p0.x*sin((k)*DEG2RAD(alpha))+p0.y*cos((k)*DEG2RAD(alpha))) +(show.rows-1)/2 );
      poly[0][2] = cv::Point(p0.x*cos((k+1)*DEG2RAD(alpha))-p0.y*sin((k+1)*DEG2RAD(alpha)) + (show.rows-1)/2, -(p0.x*sin((k+1)*DEG2RAD(alpha))+p0.y*cos((k+1)*DEG2RAD(alpha))) +(show.rows-1)/2 );
      poly[0][3] = center_point;
      const cv::Point* ppt[1] = {poly[0]};
      int npt[] = {4};
      cv::fillPoly(sector_block, ppt, npt, 1, cv::Scalar(0, 0, 255), 8);
      }
    }
  }
  cv::addWeighted(show, 1, sector_block, alpha_blend/2, 0, show);

}

void findValleyBorders(const std::vector<unsigned> &masked_hist,
                       std::vector<int>            *k_l,
                       std::vector<int>            *k_r
                      )
{
  k_l->clear();
  k_r->clear();

  unsigned current_val = masked_hist[0];
  for(unsigned i = 1; i < masked_hist.size(); ++i)
  {
    if(masked_hist[i] != current_val)
    {
      if(masked_hist[i] == 1)
      {
        k_r->push_back(i);
        current_val = 1;
      }
      else
      {
        k_l->push_back(i);
        current_val = 0;
      }
    }
  }

  if(k_r->size() != k_l->size() && masked_hist[0] == 0)
    k_l->insert(k_l->begin(), 0);

  if(k_r->size() != k_l->size() && masked_hist[0] == 1)
    k_r->push_back(0);

  if(masked_hist[0] == masked_hist[masked_hist.size()-1] && masked_hist[0] == 0 && k_r->size() != 0)
  {
    k_r->push_back(k_r->front());
    k_r->erase(k_r->begin());
  }
}

void findCandidateDirections(unsigned                 s,
                             float                    k_target,
                             const std::vector<int>   &k_l,
                             const std::vector<int>   &k_r,
                             std::vector<float>       *c
                            )
{
  c->clear();
  static const unsigned s_max = 8; // Min number of sectors for a wide valley

  for(unsigned i = 0; i < k_l.size(); ++i)
  {
    if(k_l.at(i) > k_r.at(i))
    {
      unsigned valley_size = k_r.at(i) - k_l.at(i) + s;
      if(valley_size <= s_max)
      {
        float c_tmp = (k_r.at(i) + k_l.at(i) - s) / 2.0;
        if(c_tmp < 0)
          c->push_back(c_tmp + s);
        else
          c->push_back(c_tmp);
      }
      else
      {
        float c_tmp = k_r.at(i) - (s_max / 2.0);
        if(c_tmp < 0)
          c->push_back(c_tmp + s);
        else
          c->push_back(c_tmp);

        c_tmp = k_l.at(i) + (s_max / 2.0);
        if(c_tmp >= s)
          c->push_back(c_tmp - s);
        else
          c->push_back(c_tmp);

        if((k_target < k_r.at(i) && k_target + s > k_l.at(i)) || (k_target > k_l.at(i) && k_target < k_r.at(i) + s))
          c->push_back(k_target);
      }
    }
    else
    {
      unsigned valley_size = k_r.at(i) - k_l.at(i);
      if(valley_size <= s_max)
        c->push_back((k_r.at(i) + k_l.at(i)) / 2.0);
      else
      {
        c->push_back(k_r.at(i) - (s_max / 2.0));
        c->push_back(k_l.at(i) + (s_max / 2.0));
        if(k_target > k_l.at(i) && k_target < k_r.at(i))
          c->push_back(k_target);
      }
    }
  }
}

void calculateCost(unsigned                    s,
                   unsigned                    alpha,
                   float                       k_target,
                   const std::vector<float>    &c,
                   const std::vector<float>    &mu,
                   const std::vector<unsigned> &masked_hist,
                   float                       *k_d
                  )
{
  static float prev_k_d = 0.0; // Previous direction of motion
  int drawflag = 0;
  if(c.size() != 0)
  {
    float g = 16384.0;
    for(auto &steering_dir : c)
    {
      float tmp_g = mu[0] * deltaC(steering_dir, k_target,                    s) +
                    mu[1] * deltaC(steering_dir, rpy.vector.z/DEG2RAD(alpha), s) +
                    mu[2] * deltaC(steering_dir, prev_k_d,                    s);
      if(tmp_g < g)
      {
        *k_d = steering_dir;
        g = tmp_g;
      }
    }
  }
  else
  {
    if(masked_hist[0] == 0)
    {
      *k_d = k_target;
    }
    else
    {
      *k_d = std::nanf("");
    }
  }

  prev_k_d = *k_d;

  if(drawflag!=1){
  int hist_center = (show.rows-1)/2;
  int radius_circle = round((float)CAMERARANGE*1.5/RESOLUTION_M)*upscalar;
  cv::Point p0 = cv::Point(0, radius_circle);
  cv::Point center_point = cv::Point(hist_center, hist_center);
  cv::Point p1 = cv::Point(p0.x*cos((*k_d+0.5)*DEG2RAD(alpha))-p0.y*sin((*k_d+0.5)*DEG2RAD(alpha)) + hist_center, -(p0.x*sin((*k_d+0.5)*DEG2RAD(alpha))+p0.y*cos((*k_d+0.5)*DEG2RAD(alpha))) +hist_center );
  line(show, cv::Point(hist_center, hist_center), p1, cv::Scalar(0, 255, 255), 1*upscalar/2, 8);
  }
}


bool blocked(float xt, // x coordinate of trajectory center
             float yt, // y coordinate of trajectory center
             float xc, // x coordinate of active cell
             float yc, // y coordinate of active cell
             float r   // Turning + safety radius
            )
{
  return ((pow(xt-xc,2)+pow(yt-yc,2))*pow(RESOLUTION_M, 2) < pow(r, 2)); // Returns true if circles overlap
}

float deltaC(float    c1,
             float    c2,
             unsigned s
            )
{
  return std::min(std::min(std::fabs(c1-c2-s), std::fabs(c1-c2+s)), std::fabs(c1-c2));
}

bool isBetweenRad(float start,
                  float end,
                  float mid
                 )
{
  start = wrapTo2Pi(start);
  end = wrapTo2Pi(end);
  mid = wrapTo2Pi(mid);

  end = (end - start) < 0.0f ? end - start + (2*C_PI) : end - start; // WrapTo2Pi
  mid = (mid - start) < 0.0f ? mid - start + (2*C_PI) : mid - start;
  return (mid < end);
}
