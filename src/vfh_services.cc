#include "ros/ros.h"
#include "uav_nav/VFHLookUpTables.h"
#include "uav_nav/uav_nav.h"

bool generateLUTs(uav_nav::VFHLookUpTables::Request  &req,
                  uav_nav::VFHLookUpTables::Response &res)
{
  const float b = 0.1;                                       // Scaler to linearize cell distance from RCP
  const float a = 1 + b*pow((req.size-1)/2*RESOLUTION_M, 2); // Scaler to linearize cell distance from RCP
  int x0        = floor(req.size/2);                         // Robot x coordinate in active window
  int y0        = floor(req.size/2);                         // Robot y coordinate in active window

  for(int i = 0; i < req.size; i++) // rows
  {
    for(int j = 0; j < req.size; j++) // columns
    {
      float dist = sqrt(pow((x0-i), 2)+pow((y0-j), 2))*RESOLUTION_M;
      res.beta.push_back(wrapToPi(atan2(j-y0,i-x0)+C_PI));
      res.dist.push_back(a-(b*pow(dist, 2)));
      res.gamma.push_back(asin(req.radius/dist)); // Returns nan if req.radius > dist
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vfh_services");
  ros::NodeHandle nh;

  ros::ServiceServer vfh_luts = nh.advertiseService("uav_nav/vfh_luts", generateLUTs);
  ros::spin();

  return 0;
}
