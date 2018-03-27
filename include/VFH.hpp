#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

// Callbacks
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

// Functions
void get_borders();
void get_candidates();
void cost_func();
void publish_cmd();
double delta_c(double c1, double c2);
double wrap_to_pi(double theta);
