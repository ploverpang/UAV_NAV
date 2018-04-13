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
void getLUTs(int size, float radius, std::vector<float> *beta, std::vector<float> *dist_scaled, std::vector<float> *enlarge);
void getTargetDir(unsigned alpha, const std::vector<float> &target, float *k_target);
void fillHistogramGrid(sensor_msgs::LaserScan msg);
void shiftHistogramGrid();
void binaryHist(unsigned s, unsigned alpha, float t_high, float t_low, const std::vector<float> &beta, const std::vector<float> &dist_scaled, const std::vector<float> &enlarge, std::vector<unsigned> *h);
void maskedPolarHist(unsigned alpha, float r_enl, float max_rot_vel, const std::vector<float> &beta, const std::vector<unsigned> &h, std::vector<unsigned> *masked_hist);
void findValleyBorders(const std::vector<unsigned> &masked_hist, std::vector<int> *k_l, std::vector<int> *k_r);
void findCandidateDirections(unsigned s, float k_target, const std::vector<int> &k_l, const std::vector<int> &k_r, std::vector<float> *c);
void calculateCost(unsigned s, unsigned alpha, float k_target, const std::vector<float> &c, const std::vector<float> &mu, const std::vector<unsigned> &masked_hist, float *k_d, unsigned *vel_flag);
void ctrlVelCmd(const std::vector<float> &target_xy, unsigned *vel_flag, float *lin_vel);
void publishCtrlCmd(float k_d, float lin_vel, float max_rot_vel, unsigned alpha);
bool blocked(float xt, float yt, float xc, float yc, float r);
float deltaC(float c1, float c2, unsigned s);
bool isBetweenRad(float start, float end, float mid);

#endif // UAVNAV_VFH_H_
