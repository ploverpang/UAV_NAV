#include "VFH.hpp"

// Inputs
const int s = 10; // Number of angular sectors
double target[2] = {10, 10}; // Target point for the drone

// Vars
ros::Publisher vel_cmd_pub;
geometry_msgs::PointStamped local_position;
geometry_msgs::Vector3Stamped rpy;
const unsigned alpha = 360 / s;
const unsigned s_max = 16;
double mu[3] = {5,2,2};
double k_d, prev_k_d, k_target = 0.0;
std::vector<int> k_r;
std::vector<int> k_l;
std::vector<double> c;

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
    cost_func();
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
  std::cout<<"angle_diff: "<<k_target<<std::endl;
}

void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	rpy = *msg;
}

// Functions
void get_borders(){
  k_l.clear();
  k_r.clear();

  unsigned current_val = masked_hist[0];
  for(unsigned i = 1; i < s; i++){
    if(masked_hist[i] != current_val){
      if(masked_hist[i] == 1){
        k_r.push_back(i);
        current_val = 1;
      } else{
        k_l.push_back(i);
        current_val = 0;
      }
    }
  }

  if(k_r.size() != k_l.size() && masked_hist[0] == 0){
    k_l.insert(k_l.begin(), 0);
  }
  if(k_r.size() != k_l.size() && masked_hist[0] == 1){
    k_r.push_back(0);
  }
  if(masked_hist[0] == masked_hist[s-1] && masked_hist[0] == 0 && k_r.size() != 0){
    k_r.push_back(k_r.front());
    k_r.erase(k_r.begin());
  }

  for(const double &hist_ptr : k_l){
    std::cout << hist_ptr << std::endl;
  }
  std::cout<<std::endl<<std::endl;
  for(const double &hist_ptr : k_r){
    std::cout << hist_ptr << std::endl;
  }
  std::cout<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
}

void get_candidates(){
  c.clear();
  for(unsigned i = 0; i < k_l.size(); i++){
    if(k_l.at(i) > k_r.at(i)){
      unsigned valley_size = k_r.at(i) - k_l.at(i) + s;
      if(valley_size <= s_max){
        double c_tmp = (k_r.at(i) + k_l.at(i) - s) / 2.0;
        if(c_tmp < 0){
          c.push_back(c_tmp + s);
        } else{
          c.push_back(c_tmp);
        }
      } else{
        double c_tmp = k_r.at(i) - (s_max / 2.0);
        if(c_tmp < 0){
          c.push_back(c_tmp + s);
        } else{
          c.push_back(c_tmp);
        }
        c_tmp = k_l.at(i) + (s_max / 2.0);
        if(c_tmp >= s){
          c.push_back(c_tmp - s);
        } else{
          c.push_back(c_tmp);
        }
        if((k_target < k_r.at(i) && k_target + s > k_l.at(i)) || (k_target > k_l.at(i) && k_target < k_r.at(i) + s)) {
          c.push_back(k_target);
        }
      }
    } else{
      unsigned valley_size = k_r.at(i) - k_l.at(i);
      if(valley_size <= s_max){
        c.push_back((k_r.at(i) + k_l.at(i)) / 2.0);
      } else{
        c.push_back(k_r.at(i) - (s_max / 2.0));
        c.push_back(k_l.at(i) + (s_max / 2.0));
        if(k_target > k_l.at(i) && k_target < k_r.at(i)){
          c.push_back(k_target);
        }
      }
    }
  }
  for(auto &pp : c){
    std::cout<<"c: "<<pp<<std::endl;
  }
}

void cost_func(){
  if(k_l.size() != 0 && k_r.size() != 0){
    double g = 2222222.0;
    for(auto &steering_dir : c){
      double tmp_g = mu[0] * delta_c(steering_dir, k_target) +
                     mu[1] * delta_c(steering_dir, RAD2DEG(rpy.vector.z)/alpha) +
                     mu[2] * delta_c(steering_dir, prev_k_d);
      std::cout<<"g: "<<tmp_g<<std::endl;
      if(tmp_g < g){
        k_d = steering_dir;
        g = tmp_g;
      }
    }
    prev_k_d = k_d;
  }
}

void publish_cmd() {
  double theta = wrap_to_pi(DEG2RAD(k_d * alpha));
  std::cout<<"theta: "<<theta<<std::endl;

  geometry_msgs::TwistStamped vel_cmd;
  vel_cmd.header.stamp = ros::Time::now();
  vel_cmd.header.frame_id = "vfh_vel_cmd";
  vel_cmd.twist.linear.x = 1;
  vel_cmd.twist.angular.z = theta;
  vel_cmd_pub.publish(vel_cmd);
}

double delta_c(double c1, double c2){
  return std::min(std::min(fabs(c1-c2-s), fabs(c1-c2+s)),fabs(c1-c2));
}

double wrap_to_pi(double theta){
  theta = std::fmod(theta + C_PI, 2*C_PI);
  if(theta < 0) theta += 2 * C_PI;
  return theta - C_PI;
}
