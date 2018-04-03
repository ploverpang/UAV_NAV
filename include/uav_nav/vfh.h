#ifndef UAVNAV_VFH_H_
#define UAVNAV_VFH_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "uav_nav/VFHLookUpTables.h"
#include "uav_nav/uav_nav.h"

// Callbacks
void localPositionCb(const geometry_msgs::PointStamped::ConstPtr& msg);
void velocityCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void RPYCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void laserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg);

// Functions
void getTargetDir(unsigned alpha, std::vector<float> target, float &k_target);
void getLUTs(int size, float radius, std::vector<float> &beta, std::vector<float> &dist_scaled, std::vector<float> &enlarge);
void fillHistogramGrid(sensor_msgs::LaserScan msg_laser);
void shiftHistogramGrid(geometry_msgs::PointStamped msg_pos);
void binaryHist(unsigned s, unsigned alpha, float t_high, float t_low, std::vector<float> beta, std::vector<float> dist_scaled, std::vector<float> enlarge, std::vector<unsigned> &h);
void maskedPolarHist(unsigned alpha, float r_enl, std::vector<float> beta, std::vector<unsigned> h, std::vector<unsigned> &masked_hist);
void findValleyBorders(std::vector<unsigned> masked_hist, std::vector<int> &k_l, std::vector<int> &k_r);
void findCandidateDirections(unsigned s, float k_target, std::vector<int> k_l, std::vector<int> k_r, std::vector<float> &c);
void calculateCost(unsigned s, unsigned alpha, float k_target, std::vector<float> c, std::vector<float> mu, std::vector<unsigned> masked_hist, float &k_d);
void publishCtrlCmd(float k_d, unsigned alpha);
float deltaC(float c1, float c2, unsigned s);
float wrapToPi(float angle);
float wrapTo2Pi(float angle);
bool blocked(float xt, float yt, float yc, float xc, float r);
bool checkRight(float y, float b, float r);
bool checkLeft(float y, float b, float l);
bool inRange(unsigned alpha, int x, float th_l, float th_r, float yaw);

#endif // UAVNAV_VFH_H_
