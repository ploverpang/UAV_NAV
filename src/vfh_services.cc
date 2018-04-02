#include "ros/ros.h"
#include "uav_nav/VFHLookUpTables.h"

bool GenerateLUTs(uav_nav::VFHLookUpTables::Request  &req,
                  uav_nav::VFHLookUpTables::Response &res)
{
  const float b = 1;
  const float a = 1 + b*pow((req.size-1)/2, 2);
  int x0 = floor(req.size/2);
  int y0 = floor(req.size/2);
  for(int i = 0; i < req.size; i++) //row
  {
    for(int j = 0; j < req.size; j++) // column
    {
      res.beta.push_back(atan2(j-y0,i-x0) + 3.14159);
      res.dist.push_back(a - b * pow(sqrt(pow((x0-i),2) + pow((y0-j),2)),2));
      res.gamma.push_back(asin(req.radius / sqrt(pow((x0-i),2) + pow((y0-j),2))));
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vfh_services");
  ros::NodeHandle nh;

  ros::ServiceServer vfh_luts = nh.advertiseService("uav_nav/vfh_luts", GenerateLUTs);
  ros::spin();

  return 0;
}
