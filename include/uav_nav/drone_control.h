#ifndef UAVNAV_DRONECONTROL_H_
#define UAVNAV_DRONECONTROL_H_

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// Messages
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// DJI OSDK
#include "dji_sdk/dji_sdk.h"

// DJI OSDK services
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneTaskControl.h>

#include "uav_nav/uav_nav.h"
#include <uav_nav/Steering.h>

// Initialize drone (setup, obtain control, take off)
bool isM100();
bool setLocalPositionRef();
bool obtainControl(bool b);
bool takeoffLand(int task);
bool monitoredTakeOff();
bool monitoredLanding();

// Callbacks
void flightStatusCb(const std_msgs::UInt8::ConstPtr&);
void GPSHealthCb(const std_msgs::UInt8::ConstPtr&);
void attitudeCb(const geometry_msgs::QuaternionStamped::ConstPtr&);
void heightCb(const std_msgs::Float32::ConstPtr&);
void localPositionCb(const geometry_msgs::PointStamped::ConstPtr&);
void interruptCb(const std_msgs::UInt8::ConstPtr&);
void steeringDirCb(const uav_nav::Steering::ConstPtr&);
void maskedSpeedCb(const std_msgs::UInt8::ConstPtr& msg);

// Publishers
void execCmd();
void quatToEuler();
bool setAltitude(float);

#endif // UAVNAV_DRONECONTROL_H_
