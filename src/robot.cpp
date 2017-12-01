#include <ros/ros.h> 
#include <pioneer/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot");
  ros::NodeHandle nh;
  Pioneer* pioneer = new Pioneer(nh);
  ros::spin();
}
