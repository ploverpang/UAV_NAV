#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Global variables
std_msgs::UInt8 interrupt_signal;
ros::Publisher signal_interrupt;
double threshold_sec = 0.5;
float ultrasonic_threshold = 2;
float ultrasonic_intensity = 1;
bool safetyFlag = true;
bool first = false;

// Timer resets
double ultrasonic_reset;
double laser_scan_reset;
double rpy_reset;
double local_pos_reset;
double velocity_reset;
double vel_cmd_reset;

// Callback Timer counters
double ultrasonic_counter, laser_scan_counter, rpy_counter, local_pos_counter, velocity_counter, vel_cmd_counter;

// Callback functions
void ultrasonic_callback(const sensor_msgs::LaserScan& msg)
{
	ultrasonic_reset = ros::Time::now().toSec();
	// Check range values
	for(int n = 1; n < 5; ++n)
	{
	  if(msg.ranges[n] < ultrasonic_threshold && msg.ranges[n] > 0 && msg.intensities[n] == ultrasonic_intensity)
	  {
	    safetyFlag = true;
	  }
	}

	// If range values were too close
	if(safetyFlag && !first)
	{
	   ROS_ERROR("Interrupt signal was triggered. Range reading were %f, %f, %f and %f", msg.ranges[1], msg.ranges[2],msg.ranges[3], msg.ranges[4]);
	   interrupt_signal.data = 2;
	   signal_interrupt.publish(interrupt_signal);
	   first=true;
	}
	else if(!safetyFlag && first)
	{
	   interrupt_signal.data = 0;
	   signal_interrupt.publish(interrupt_signal);
	   ROS_INFO("Range reading were OK, resetting interrupt signal");
	   first = false;
	}

	safetyFlag = false;
}

void laser_scan_callback(const sensor_msgs::LaserScan& msg)
{
	laser_scan_reset = ros::Time::now().toSec();
}

void rpy_callback(const geometry_msgs::Vector3Stamped& msg)
{
	rpy_reset = ros::Time::now().toSec();
}

void local_pos_callback(const geometry_msgs::PointStamped& msg)
{
	local_pos_reset = ros::Time::now().toSec();
}

void velocity_callback(const geometry_msgs::Vector3Stamped& msg)
{
	velocity_reset = ros::Time::now().toSec();
}

void vel_cmd_callback(const geometry_msgs::TwistStamped& msg)
{
	vel_cmd_reset = ros::Time::now().toSec();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "safety_interrupt");
	ros::NodeHandle nh;
	ros::Duration(20.5).sleep();
	vel_cmd_reset = ros::Time::now().toSec();
	velocity_reset = ros::Time::now().toSec();
	rpy_reset = ros::Time::now().toSec();
	ultrasonic_reset = ros::Time::now().toSec();
	laser_scan_reset = ros::Time::now().toSec();
	local_pos_reset = ros::Time::now().toSec();

	// Subscribers
	ros::Subscriber ultrasonic_sub = nh.subscribe("uav_nav/guidance/ultrasonic",      1, &ultrasonic_callback); // sensor_feedback
	ros::Subscriber laser_scan_sub = nh.subscribe("uav_nav/laser_scan_from_depthIMG", 1, &laser_scan_callback); // depth_generation
  ros::Subscriber rpy_sub        = nh.subscribe("uav_nav/roll_pitch_yaw",           1, &rpy_callback); // drone_control
 	ros::Subscriber loc_pos_sub    = nh.subscribe("dji_sdk/local_position",           1, &local_pos_callback); // dji_sdk
 	ros::Subscriber vel_sub        = nh.subscribe("dji_sdk/velocity",                 1, &velocity_callback);
	ros::Subscriber vel_cmd_pub    = nh.subscribe("uav_nav/vel_cmd",                  1, &vel_cmd_callback); // vfh

	// Publisher
	signal_interrupt = nh.advertise<std_msgs::UInt8>("uav_nav/signal_interrupt", 1);

	interrupt_signal.data = 0;

	// check resets
	while(ros::ok())
	{
		if(interrupt_signal.data != 2)
		{
  		vel_cmd_counter    = ros::Time::now().toSec() - vel_cmd_reset;
  		velocity_counter   = ros::Time::now().toSec() - velocity_reset;
  		rpy_counter        = ros::Time::now().toSec() - rpy_reset;
  		ultrasonic_counter = ros::Time::now().toSec() - ultrasonic_reset;
  		laser_scan_counter = ros::Time::now().toSec() - laser_scan_reset;
  		local_pos_counter  = ros::Time::now().toSec() - local_pos_reset;

  		if(vel_cmd_counter > threshold_sec    ||
         velocity_counter > threshold_sec   ||
         rpy_counter > threshold_sec        ||
         ultrasonic_counter > threshold_sec ||
         laser_scan_counter > threshold_sec ||
         local_pos_counter > threshold_sec)
      {
        interrupt_signal.data = 1;
  			ROS_ERROR("Data logging problem, times: %f, %f, %f, %f, %f and %f",
                  vel_cmd_counter,
                  velocity_counter,
                  rpy_counter,
                  ultrasonic_counter,
                  laser_scan_counter,
                  local_pos_counter);
      }
  		else
      {
        interrupt_signal.data = 0;
      }

  		signal_interrupt.publish(interrupt_signal);
		}

		ros::spinOnce();
	}

	return 0;
}
