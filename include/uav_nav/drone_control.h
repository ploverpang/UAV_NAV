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
bool is_M100();
bool set_local_position();
bool obtain_control(bool b);
bool takeoff_land(int task);
bool monitored_takeoff();

// Callbacks
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void vel_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

// Publishers
void send_vel_command(geometry_msgs::TwistStamped cmd);
void quat_to_eul();

#endif // UAVNAV_DRONECONTROL_H_
