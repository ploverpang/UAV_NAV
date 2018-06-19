#include "uav_nav/drone_control.h"

// Global variables
ros::ServiceClient               query_version_service;
ros::ServiceClient               set_loc_pos_ref_service;
ros::ServiceClient               sdk_ctrl_authority_service;
ros::ServiceClient               drone_task_service;
ros::Publisher 				           ready_pub;                         //
ros::Publisher 				           ctrl_vel_cmd_pub;                  // Velocity control command sent to the FC
ros::Publisher 				           rpy_pub;                           // Publish roll, pitch, yaw in radians
geometry_msgs::PointStamped      local_position;                    // Local position offset in FLU frame
geometry_msgs::TwistStamped	     vel_cmd;
geometry_msgs::QuaternionStamped attitude_state;
uav_nav::Steering                steering_dir;
float	 		                       altitude;
float                            height                      = 0;
uint8_t                          flight_status               = 255; // Enum representing drone state upon take off
uint8_t                          current_gps_health          = 0;   // Number of GPS satellite connections
int 				                     ctrl_state	          	     = 0;   // State machine controller
geometry_msgs::Vector3Stamped    loc_rpy;


void FSM()
{
  switch(ctrl_state) {
    case 0:
      break;

    case 1:
      if(isM100() && setLocalPositionRef())
      {
        bool ready = obtainControl(true) ? monitoredTakeOff() : false;
        if(ready)
        {
          ctrl_state = 2;
        }
        else
        {
          ROS_ERROR("Take-off failed! || Could not obtain authority.");
          ros::shutdown();
        }
      }
      else
      {
        ROS_ERROR("Only the M100 is supported! || Could not set local position reference.");
        ros::shutdown();
      }
      break;

    case 2:
      ctrl_state = setAltitude(altitude) ? 3 : 90;
      break;

    case 3:
    {
      std_msgs::Bool r;
      r.data = true;
      ready_pub.publish(r);
      ctrl_state = 4;
      ros::spinOnce();
      break;
    }

    case 90:
      bool landed = monitoredLanding() ? obtainControl(false) : false;
      if(landed)
      {
        ROS_INFO("Drone landed and control is released.\nShutting down drone_control...");
      }
      else
      {
        ROS_ERROR("Landing failed! Take over control manually.");
      }

      ros::shutdown();
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_control");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh_("~");
  ros::Duration(10).sleep();

  // Parameters
  private_nh_.param("/drone_control/alt", altitude, 2.5f);

  // Services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>	  ("dji_sdk/query_drone_version");
  set_loc_pos_ref_service    = nh.serviceClient<dji_sdk::SetLocalPosRef>		  ("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>	("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>	  ("dji_sdk/drone_task_control");

  // Subscribers
  ros::Subscriber flight_status_sub = nh.subscribe("dji_sdk/flight_status",        1, &flightStatusCb);
  ros::Subscriber gps_health_sub    = nh.subscribe("dji_sdk/gps_health",           1, &GPSHealthCb);
  ros::Subscriber attitude_sub      = nh.subscribe("dji_sdk/attitude",             1, &attitudeCb);
  ros::Subscriber height_takeoff    = nh.subscribe("dji_sdk/height_above_takeoff", 1, &heightCb);
  ros::Subscriber loc_pos_sub       = nh.subscribe("dji_sdk/local_position",       1, &localPositionCb);
  ros::Subscriber steering_dir_sub  = nh.subscribe("uav_nav/steering_dir",         1, &steeringDirCb);
  ros::Subscriber interrupt_pub     = nh.subscribe("uav_nav/signal_interrupt",     1, &interruptCb);

  // Publishers
  ready_pub         = nh.advertise<std_msgs::Bool>("uav_nav/dc_ready", 1);
  ctrl_vel_cmd_pub  = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
  rpy_pub           = nh.advertise<geometry_msgs::Vector3Stamped>("uav_nav/roll_pitch_yaw", 1);

  while(ros::ok())
  {
    FSM();
    ros::spinOnce();
  }

  return 0;
}

// Services
bool isM100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  return (query.response.version == DJISDK::DroneFirmwareVersion::M100_31);
}

bool setLocalPositionRef()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_loc_pos_ref_service.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}

bool obtainControl(bool b)
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = b ? 1 : 0;
  sdk_ctrl_authority_service.call(authority);

  return authority.response.result;
}

bool takeoffLand(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = task;
  drone_task_service.call(droneTaskControl);

  return droneTaskControl.response.result;
}

bool monitoredTakeOff()
{
  static const ros::Time start_time = ros::Time::now();

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    return false;

  ROS_DEBUG("M100 taking off!");
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  while(ros::Time::now() - start_time < ros::Duration(10) &&
        flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR &&
        height < 0.7)
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || height < 0.7)
  {
    ROS_ERROR("Take-off failed.\nHeight above takeoff: %f", height);
    return false;
  }

  ROS_DEBUG("Successful take-off! Height above takeoff is: %f", height);
  ros::spinOnce();
  return true;
}

bool monitoredLanding()
{
  static const ros::Time start_time = ros::Time::now();

  if(!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
    return false;

  ROS_DEBUG("M100 landing!");
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  while(ros::Time::now() - start_time < ros::Duration(10) &&
        flight_status != DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
  {
    ROS_ERROR("Landing failed.");
    return false;
  }

  ROS_INFO("Successful landing!");
  ros::spinOnce();
  return true;
}

// Callbacks
void flightStatusCb(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void GPSHealthCb(const std_msgs::UInt8::ConstPtr& msg)
{
  current_gps_health = msg->data;

  if (current_gps_health <= 3)
  {
    ctrl_state = 0;
    ROS_ERROR("Cannot execute local position control. Not enough GPS satellites");
  }
}

void attitudeCb(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
	attitude_state = *msg;

  quatToEuler();
}

void heightCb(const std_msgs::Float32::ConstPtr& msg)
{
  height = msg->data;
}

void localPositionCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position = *msg;
}

void interruptCb(const std_msgs::UInt8::ConstPtr& msg) {
  switch (msg->data)
  {
    case 0:
      ctrl_state = 1;
      ROS_DEBUG("Safety OK");
    case 1:
      ctrl_state = 0;
      ROS_ERROR("System malfunction");
      break;
    case 2:
      ROS_WARN("Object inside safety threshold");
      ctrl_state = 2;
      break;
  }
}

void steeringDirCb(const uav_nav::Steering::ConstPtr& msg)
{
  steering_dir = *msg;

  execCmd();
}

void execCmd()
{
  static const float max_vel       = 1.5;
  static const float target_radius = 3.0;
  static const float max_rot_vel = 1.0;
  float lin_vel = 0.0;

  float target_distance = sqrt(pow(steering_dir.target[0]-local_position.point.x, 2) +
                               pow(steering_dir.target[1]-local_position.point.y, 2));
  if(target_distance > target_radius)
    lin_vel = max_vel;
  else if (target_distance > 1.5)
    lin_vel = (target_distance/target_radius) * max_vel;
  else{
    bool landed = monitoredLanding() ? obtainControl(false) : false;
    if(landed)
    {
      ROS_INFO("Drone landed and control is released.\nShutting down drone_control...");
    }
    else
    {
      ROS_ERROR("Landing failed! Take over control manually.");
    }

    ros::shutdown();
  }


  /*switch (*vel_flag)
  {
    case 1:
      *lin_vel *= 0.5;
      break;
    case 2:
      *lin_vel = 0;
      if(sqrt(pow(velocity.vector.x,2)+pow(velocity.vector.y,2)) < 0.1)
        *vel_flag = 0;
      break;
  }*/


  //float yawrate = wrapToPi(DEG2RAD(steering_dir.steering_dir * steering_dir.alpha));
  float yawrate;
  if(abs(steering_dir.k_target-steering_dir.steering_dir) < 0.0001)
    yawrate = wrapToPi(DEG2RAD(steering_dir.k_target * steering_dir.alpha));
  else
    yawrate = wrapToPi(DEG2RAD(steering_dir.steering_dir * steering_dir.alpha)-loc_rpy.vector.z);
  if(yawrate > max_rot_vel) {
    yawrate = std::copysign(yawrate, max_rot_vel);
  }

  /*geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp    = ros::Time::now();
  vel_cmd.header.frame_id = "vfh_vel_cmd";
  vel_cmd.twist.linear.x  = lin_vel;
  vel_cmd.twist.linear.y  = target_reached;
  vel_cmd.twist.angular.z = yawrate;
  vel_cmd_pub.publish(vel_cmd);*/

  ROS_INFO("lin_vel: %f, yawrate: %f", lin_vel, yawrate);

  sensor_msgs::Joy ctrl_vel_yawrate;
  uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                  DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_RATE            |
                  DJISDK::HORIZONTAL_BODY     |
                  DJISDK::STABLE_ENABLE
                 );
  switch(ctrl_state)
  {
    case 0:	break;
    case 1:
      ctrl_vel_yawrate.axes.push_back(lin_vel);
      ctrl_vel_yawrate.axes.push_back(0);
      ctrl_vel_yawrate.axes.push_back(0);
      ctrl_vel_yawrate.axes.push_back(yawrate);
      ctrl_vel_yawrate.axes.push_back(flag);

      ctrl_vel_cmd_pub.publish(ctrl_vel_yawrate);
      break;
    case 2:	//Rotate
      ctrl_vel_yawrate.axes.push_back(0);
      ctrl_vel_yawrate.axes.push_back(0);
      ctrl_vel_yawrate.axes.push_back(0);
      ctrl_vel_yawrate.axes.push_back(1);
      ctrl_vel_yawrate.axes.push_back(flag);

      ctrl_vel_cmd_pub.publish(ctrl_vel_yawrate);
      break;
  }
}

void quatToEuler()
{
  geometry_msgs::Vector3Stamped rpy;
  rpy.header.stamp    = ros::Time::now();
  rpy.header.frame_id = "RPY";

  tf::Quaternion q = tf::Quaternion(attitude_state.quaternion.x,
                                    attitude_state.quaternion.y,
                                    attitude_state.quaternion.z,
                                    attitude_state.quaternion.w);
  tf::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

  rpy.vector.z = wrapToPi(rpy.vector.z - C_PI/2); // Proper FLU - 'x' towards North = 0 yaw; yaw > 0 ccw

  loc_rpy = rpy;

  rpy_pub.publish(rpy);
}

bool setAltitude(float alt)
{
  static const ros::Time start_time = ros::Time::now();

  while(std::fabs(alt-height) > 0.35 &&
        ros::Time::now() - start_time < ros::Duration(20))
  {
    float vel_z = std::fabs(alt-height) > 0.5 ? std::copysign(0.5, alt-height) : alt-height;

    sensor_msgs::Joy ctrl_vel_yawrate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_BODY     |
                    DJISDK::STABLE_ENABLE
                   );
    ctrl_vel_yawrate.axes.push_back(0);
    ctrl_vel_yawrate.axes.push_back(0);
    ctrl_vel_yawrate.axes.push_back(vel_z);
    ctrl_vel_yawrate.axes.push_back(0);
    ctrl_vel_yawrate.axes.push_back(flag);

    ctrl_vel_cmd_pub.publish(ctrl_vel_yawrate);

    ros::spinOnce();
  }

  if(std::fabs(alt-height) < 0.35)
  {
    ROS_DEBUG("Drone reached altitude above takeoff: %f", height);
    return true;
  }
  else
  {
    ROS_DEBUG("Drone did not reach desired altitude above takeoff: %f", alt);
    return false;
  }
}
