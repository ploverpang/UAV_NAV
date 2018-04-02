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

#define CAMERARANGE 10
#define RESOLUTION_M 0.5
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

// Callbacks
void LocalPositionCb(const geometry_msgs::PointStamped::ConstPtr& msg);
void VelocityCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void RollPitchYawCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void LaserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg);

// Functions
void GetTargetDir(unsigned alpha, std::vector<float> target, float &k_target);
void GetLUTs(int size, float radius, std::vector<float> &beta, std::vector<float> &dist_scaled, std::vector<float> &enlarge);
void FillHistogramGrid(sensor_msgs::LaserScan msg_laser);
void ShiftHistogramGrid(geometry_msgs::PointStamped msg_pos);
void BinaryHist(unsigned s, unsigned alpha, float t_high, float t_low, std::vector<float> beta, std::vector<float> dist_scaled, std::vector<float> enlarge, std::vector<unsigned> &h);
void MaskedPolarHist(unsigned alpha, float r_enl, std::vector<float> beta, std::vector<unsigned> h, std::vector<unsigned> &masked_hist);
void FindValleyBorders(std::vector<unsigned> masked_hist, std::vector<int> &k_l, std::vector<int> &k_r);
void FindCandidateDirections(unsigned s, float k_target, std::vector<int> k_l, std::vector<int> k_r, std::vector<float> &c);
void CalculateCost(unsigned s, unsigned alpha, float k_target, std::vector<float> c, std::vector<float> mu, std::vector<unsigned> masked_hist, float &k_d);
void PublishCtrlCmd(float k_d, unsigned alpha);
float DeltaC(float c1, float c2, unsigned s);
float WrapToPi(float angle);
float WrapTo2Pi(float angle);
bool Blocked(float xt, float yt, float yc, float xc, float r);
bool CheckRight(float y, float b, float r);
bool CheckLeft(float y, float b, float l);
bool InRange(unsigned alpha, int x, float th_l, float th_r, float yaw);

#endif // UAVNAV_VFH_H_
