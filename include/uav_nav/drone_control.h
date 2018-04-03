#ifndef UAVNAV_DRONECONTROL_H_
#define UAVNAV_DRONECONTROL_H_

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// Messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
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

// Initialize drone (setup, obtain control, take off)
bool isM100();
bool setLocalPositionRef();
bool obtainControl(bool b);
bool takeoffLand(int task);
bool monitoredTakeOff();

// Callbacks
void flightStatusCb(const std_msgs::UInt8::ConstPtr& msg);
void GPSPositionCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void GPSHealthCb(const std_msgs::UInt8::ConstPtr& msg);
void attitudeCb(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void velCmdCb(const geometry_msgs::TwistStamped::ConstPtr& msg);

// Publishers
void sendVelCmd(geometry_msgs::TwistStamped cmd);
void quatToEuler();

#endif // UAVNAV_DRONECONTROL_H_
