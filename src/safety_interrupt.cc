#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
//#include <geometry_msgs/Vector3Stamped.h>
//#include <geometry_msgs/PointStamped.h>

// Global variables
std_msgs::UInt8 interrupt_signal;
ros::Publisher  signal_interrupt;
double          threshold_sec        = 0.5;
float           ultrasonic_threshold;
bool            safetyFlag           = true;
bool            first                = false;
bool            ready                = false;

// Counter resets
double ultrasonic_reset;
//double laser_scan_reset;
//double rpy_reset;
//double local_pos_reset;
//double velocity_reset;

// Callback counters
double ultrasonic_counter;//, laser_scan_counter, rpy_counter, local_pos_counter, velocity_counter, vel_cmd_counter;

// Callback functions
void ultrasonic_callback(const sensor_msgs::LaserScan& msg)
{
	ultrasonic_reset = ros::Time::now().toSec();

	// Check range values
	for(int n = 1; n < 5; ++n)
	{
	  if(msg.ranges[n] < ultrasonic_threshold &&
       msg.ranges[n] > 0 &&
       msg.intensities[n] == 1)
	    safetyFlag = true;
	}

	// If range values were too close
	if(safetyFlag && !first)
	{
	   ROS_ERROR("Interrupt signal was triggered. Range readings were %f, %f, %f, %f",
                msg.ranges[1],
                msg.ranges[2],
                msg.ranges[3],
                msg.ranges[4]);
	   interrupt_signal.data = 2;
	   signal_interrupt.publish(interrupt_signal);
	   first=true;
	}
	else if(!safetyFlag && first)
	{
	   interrupt_signal.data = 0;
	   signal_interrupt.publish(interrupt_signal);
	   ROS_DEBUG("Range readings were OK, resetting interrupt signal.");
	   first = false;
	}

	safetyFlag = false;
}

/*
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
*/

void readyCb(const std_msgs::Bool::ConstPtr& msg)
{
  ready = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "safety_interrupt");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh_("~");

	private_nh_.param("/safety/ultrasonic_threshold", ultrasonic_threshold, 2.f);

  ros::Subscriber ready_sub = nh.subscribe("uav_nav/dc_ready", 1, &readyCb);
  while(ros::ok() && !ready)
  {
    ros::spinOnce();
  }

  ultrasonic_reset = ros::Time::now().toSec();
  //laser_scan_reset = ros::Time::now().toSec();
	//rpy_reset = ros::Time::now().toSec();
	//local_pos_reset = ros::Time::now().toSec();
  //velocity_reset = ros::Time::now().toSec();

	// Subscribers
	ros::Subscriber ultrasonic_sub = nh.subscribe("uav_nav/guidance/ultrasonic",      1, &ultrasonic_callback);
	//ros::Subscriber laser_scan_sub = nh.subscribe("uav_nav/laser_scan_from_depthIMG", 1, &laser_scan_callback);
  //ros::Subscriber rpy_sub        = nh.subscribe("uav_nav/roll_pitch_yaw",           1, &rpy_callback);
 	//ros::Subscriber loc_pos_sub    = nh.subscribe("dji_sdk/local_position",           1, &local_pos_callback);
 	//ros::Subscriber vel_sub        = nh.subscribe("dji_sdk/velocity",                 1, &velocity_callback);

	// Publisher
	signal_interrupt = nh.advertise<std_msgs::UInt8>("uav_nav/signal_interrupt", 1);

	interrupt_signal.data = 0;

	ros::spinOnce();

	while(ros::ok())
	{
		if(interrupt_signal.data != 2)
		{
  		ultrasonic_counter = ros::Time::now().toSec() - ultrasonic_reset;
  		//laser_scan_counter = ros::Time::now().toSec() - laser_scan_reset;
      //rpy_counter        = ros::Time::now().toSec() - rpy_reset;
  		//local_pos_counter  = ros::Time::now().toSec() - local_pos_reset;
      //velocity_counter   = ros::Time::now().toSec() - velocity_reset;

  		if(ultrasonic_counter > threshold_sec /*||
         laser_scan_counter > threshold_sec ||
         rpy_counter        > threshold_sec ||
         local_pos_counter  > threshold_sec ||
         velocity_counter   > threshold_sec*/)
      {
        interrupt_signal.data = 1;
        ROS_ERROR("Ultrasonic topic is down. Delay: %f.", ultrasonic_counter);
  			/*ROS_ERROR("Data logging problem, times: %f, %f, %f, %f, %f and %f",
                  vel_cmd_counter,
                  velocity_counter,
                  rpy_counter,
                  ultrasonic_counter,
                  laser_scan_counter,
                  local_pos_counter);*/
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
