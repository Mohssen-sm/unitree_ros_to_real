#include <fstream>
#include <geometry_msgs/WrenchStamped.h>
#include "../../include/StateEstimation.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "streaming");

  StateEstimation SE("aliengo");

  ros::Rate rate(1000);

  ros::NodeHandle n;

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  };
  
  return 0;
}
