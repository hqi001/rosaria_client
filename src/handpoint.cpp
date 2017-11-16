#include <ros/ros.h> 
#include <pioneer/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "handpoint");
  ros::NodeHandle nh;
  
  Pioneer* pioneer = new Pioneer(nh);
  
  ros::Rate rate(10);
  
  while(ros::ok())
  {
    //pioneer->pubVel();
    ros::spinOnce();
    rate.sleep();
  }

}
