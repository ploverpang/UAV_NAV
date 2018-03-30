#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

// Callbacks
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void pos_callback(const geometry_msgs::PointStamped& msg_pos);
void laser_scan_callback(const sensor_msgs::LaserScan& msg_laser);

// Functions
void fillHistogramGrid(sensor_msgs::LaserScan msg_laser);
void shiftHistogramGrid(geometry_msgs::PointStamped msg_pos);
void generate_LUTs();
void binary_hist();
void masked_polar_histogram();
void get_borders();
void get_candidates();
void calc_cost();
void publish_cmd();
double delta_c(double c1, double c2);
double wrap_to_pi(double angle);
double wrap_to_2pi(double angle);
bool blocked(float xt, float yt, float yc, float xc, float r);
bool checkright(float y, float b, float r);
bool checkleft(float y, float b, float l);
bool inrange(int x, float th_l, float th_r, float yaw);
