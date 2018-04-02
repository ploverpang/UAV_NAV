#include "uav_nav/drone_control.h"

// Global variables
ros::ServiceClient               query_version_service;
ros::ServiceClient               set_loc_pos_ref_service;
ros::ServiceClient               sdk_ctrl_authority_service;
ros::ServiceClient               drone_task_service;
ros::Publisher 				           ctrl_vel_cmd_pub;
ros::Publisher 				           rpy_pub;
geometry_msgs::TwistStamped	     vel_cmd;
geometry_msgs::QuaternionStamped attitude_state;
sensor_msgs::NavSatFix           current_gps_position;
uint8_t                          flight_status               = 255;
uint8_t                          current_gps_health          = 0;
int 						                 ctrl_state 	        			 = 0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_control");
  ros::NodeHandle nh;

  // Services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>	  ("dji_sdk/query_drone_version");
  set_loc_pos_ref_service    = nh.serviceClient<dji_sdk::SetLocalPosRef>		  ("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>	("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>	  ("dji_sdk/drone_task_control");

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flight_status_sub = nh.subscribe("dji_sdk/flight_status",   1, &flight_status_callback);
  ros::Subscriber gps_pos_sub       = nh.subscribe("dji_sdk/gps_position",    1, &gps_position_callback);
  ros::Subscriber gps_health_sub    = nh.subscribe("dji_sdk/gps_health",      1, &gps_health_callback);
  ros::Subscriber attitude 		      = nh.subscribe("dji_sdk/attitude",        1, &attitude_callback);
  ros::Subscriber control_cmd_sub   = nh.subscribe("uav_nav/vel_cmd",  	      1, &vel_cmd_callback);

  // Publish the control signal
  ctrl_vel_cmd_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 1);
  rpy_pub          = nh.advertise<geometry_msgs::Vector3Stamped>("uav_nav/roll_pitch_yaw",                 1);

  bool ready = false;
  if(is_M100() && set_local_position()) {
    ready = obtain_control(true) ? monitored_takeoff() : false;
  }
  else {
    ROS_ERROR("Only the M100 is supported! || Could not set local position reference.");
  }

  if(ready) ctrl_state = 1;

  ros::spin();
  return 0;
}

// Services
bool is_M100() {
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  return (query.response.version == DJISDK::DroneFirmwareVersion::M100_31);
}

bool set_local_position() {
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_loc_pos_ref_service.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}

bool obtain_control(bool b) {
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = b ? 1 : 0;
  sdk_ctrl_authority_service.call(authority);

  return authority.response.result;
}

bool takeoff_land(int task) {
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = task;
  drone_task_service.call(droneTaskControl);

  return droneTaskControl.response.result;
}

bool monitored_takeoff() {
  ros::Time start_time = ros::Time::now();
  float home_altitude = current_gps_position.altitude;

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) return false;
  ROS_INFO("M100 taking off!");

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || (current_gps_position.altitude - home_altitude < 1.0)) {
    ROS_ERROR("Take-off failed.");
    return false;
  }
  else {
    // start_time = ros::Time::now();
    ROS_INFO("Successful take-off!");
    ros::spinOnce();
  }
  return true;
}

// Callbacks
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
  flight_status = msg->data;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;


  if (current_gps_health <= 3) {
    ctrl_state = 0;
    ROS_WARN("Cannot execute local position control. Not enough GPS satellites");
  }
  else {
    ctrl_state = 1;
  }
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
	attitude_state = *msg;

  quat_to_eul();
}

void vel_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  vel_cmd = *msg;

  switch(ctrl_state) {
    case 0:	break;
    case 1:
      send_vel_command(vel_cmd);
      break;
  }
}

void send_vel_command(geometry_msgs::TwistStamped cmd) {
  ROS_INFO("Vx: %.3f /tYaw_rate: %.3f", cmd.twist.linear.x, cmd.twist.angular.z);

  sensor_msgs::Joy controlVelYawRate;
  controlVelYawRate.axes.push_back(cmd.twist.linear.x);
  controlVelYawRate.axes.push_back(0);
  controlVelYawRate.axes.push_back(0);
  controlVelYawRate.axes.push_back(cmd.twist.angular.z);

  ctrl_vel_cmd_pub.publish(controlVelYawRate);
}

void quat_to_eul() {
  geometry_msgs::Vector3Stamped rpy;
  rpy.header.stamp = ros::Time::now();
  rpy.header.frame_id = "RPY";

  tf::Quaternion q = tf::Quaternion(attitude_state.quaternion.x,
                                    attitude_state.quaternion.y,
                                    attitude_state.quaternion.z,
                                    attitude_state.quaternion.w);
  tf::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

  rpy_pub.publish(rpy);
}
