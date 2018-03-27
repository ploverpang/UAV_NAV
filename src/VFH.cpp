#include "VFH.hpp"

// Inputs
const int s      = 10;       // Number of angular sectors
double target[2] = {10, 10}; // Target for the drone

// Vars
ros::Publisher                vel_cmd_pub;
geometry_msgs::PointStamped   local_position;
geometry_msgs::Vector3Stamped rpy;
const unsigned                alpha           = 360 / s; // Sector angle
const unsigned                s_max           = 16;      // Min number of sectors for a wide valley
double                        mu[3]           = {5,2,2}; // Cost parameters
double                        k_d             = 0.0;     // Direction of motion
double                        prev_k_d        = 0.0;     // Previous direction of motion
double                        k_target        = 0.0;     // Target direction
std::vector<int>              k_r;                       // Right borders
std::vector<int>              k_l;                       // Left borders
std::vector<double>           c;                         // Candidate directions

// DEBUGG ONLY
unsigned masked_hist[s] = {1,1,1,1,1,1,0,1,0,0};

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "vfh");
  ros::NodeHandle nh;

  //Subsriber
  ros::Subscriber loc_pos_sub = nh.subscribe("dji_sdk/local_position", 1, &local_position_callback);
  ros::Subscriber rpy_sub     = nh.subscribe("rob666/roll_pitch_yaw",  1, &rpy_callback);

  //Publishers
  vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped> ("/rob666/vel_cmd", 1);

  ros::Rate r(1); //1Hz

  while (ros::ok()) {
    get_borders();
    get_candidates();
    calc_cost();
    publish_cmd();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

// Callbacks
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	local_position = *msg;

  double target_angle = std::atan2(target[1] - local_position.point.y, target[0] - local_position.point.x);
  double angle_diff = target_angle - rpy.vector.z;
  angle_diff = wrap_to_pi(angle_diff);
  k_target = RAD2DEG(angle_diff) / alpha;
}

void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	rpy = *msg;
}

// Functions
void get_borders() {
  k_l.clear();
  k_r.clear();

  unsigned current_val = masked_hist[0];
  for(unsigned i = 1; i < s; i++) {
    if(masked_hist[i] != current_val) {
      if(masked_hist[i] == 1) {
        k_r.push_back(i);
        current_val = 1;
      } else{
        k_l.push_back(i);
        current_val = 0;
      }
    }
  }

  if(k_r.size() != k_l.size() && masked_hist[0] == 0) {
    k_l.insert(k_l.begin(), 0);
  }
  if(k_r.size() != k_l.size() && masked_hist[0] == 1) {
    k_r.push_back(0);
  }
  if(masked_hist[0] == masked_hist[s-1] && masked_hist[0] == 0 && k_r.size() != 0) {
    k_r.push_back(k_r.front());
    k_r.erase(k_r.begin());
  }
}

void get_candidates() {
  c.clear();

  for(unsigned i = 0; i < k_l.size(); i++) {
    if(k_l.at(i) > k_r.at(i)) {
      unsigned valley_size = k_r.at(i) - k_l.at(i) + s;
      if(valley_size <= s_max) {
        double c_tmp = (k_r.at(i) + k_l.at(i) - s) / 2.0;
        if(c_tmp < 0) {
          c.push_back(c_tmp + s);
        } else{
          c.push_back(c_tmp);
        }
      } else {
        double c_tmp = k_r.at(i) - (s_max / 2.0);
        if(c_tmp < 0) {
          c.push_back(c_tmp + s);
        } else {
          c.push_back(c_tmp);
        }
        c_tmp = k_l.at(i) + (s_max / 2.0);
        if(c_tmp >= s) {
          c.push_back(c_tmp - s);
        } else {
          c.push_back(c_tmp);
        }
        if((k_target < k_r.at(i) && k_target + s > k_l.at(i)) || (k_target > k_l.at(i) && k_target < k_r.at(i) + s)) {
          c.push_back(k_target);
        }
      }
    } else {
      unsigned valley_size = k_r.at(i) - k_l.at(i);
      if(valley_size <= s_max) {
        c.push_back((k_r.at(i) + k_l.at(i)) / 2.0);
      } else {
        c.push_back(k_r.at(i) - (s_max / 2.0));
        c.push_back(k_l.at(i) + (s_max / 2.0));
        if(k_target > k_l.at(i) && k_target < k_r.at(i)) {
          c.push_back(k_target);
        }
      }
    }
  }
}

void calc_cost() {
  if(k_l.size() != 0 && k_r.size() != 0) {
    double g = 16384.0;
    for(auto &steering_dir : c) {
      double tmp_g = mu[0] * delta_c(steering_dir, k_target) +
                     mu[1] * delta_c(steering_dir, RAD2DEG(rpy.vector.z)/alpha) +
                     mu[2] * delta_c(steering_dir, prev_k_d);
      if(tmp_g < g) {
        k_d = steering_dir;
        g = tmp_g;
      }
    }
    prev_k_d = k_d;
  }
}

void publish_cmd() {
  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp    = ros::Time::now();
  vel_cmd.header.frame_id = "vfh_vel_cmd";
  vel_cmd.twist.linear.x  = 1; // This needs to be correctly controlled
  vel_cmd.twist.angular.z = wrap_to_pi(DEG2RAD(k_d * alpha));
  vel_cmd_pub.publish(vel_cmd);
}

double delta_c(double c1, double c2) {
  return std::min(std::min(fabs(c1-c2-s), fabs(c1-c2+s)),fabs(c1-c2));
}

double wrap_to_pi(double angle) {
  angle = std::fmod(angle + C_PI, 2*C_PI);
  if(angle < 0) angle += 2 * C_PI;
  return angle - C_PI;
}
