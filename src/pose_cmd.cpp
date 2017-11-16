#include <ros/ros.h> 
#include <pioneer/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_cmd");
  ros::NodeHandle nh;
  
  Pioneer* pioneer = new Pioneer(nh);
  geometry_msgs::Pose2D diff;
  geometry_msgs::Twist vel_hp; 
  ros::Publisher pub_vel_hp = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel_hp", 1);
    
  vel_hp.linear.x = 0;
  vel_hp.linear.y = 0;
  vel_hp.linear.z = 0;
  vel_hp.angular.x = 0;
  vel_hp.angular.y = 0;
  vel_hp.angular.z = 0;

  ros::Rate rate(10);
  
  while(ros::ok())
  {
    diff = pioneer->getPoseDiff();
    if(diff.x > 0.2)
      vel_hp.linear.x = 0.2;
    else
      vel_hp.linear.x = 0;
    if(diff.y > 0.2)
      vel_hp.angular.z = 0.2;
    else
      vel_hp.angular.z = 0;

    ROS_INFO_STREAM(diff);
    pub_vel_hp.publish(vel_hp);

    ros::spinOnce();
    rate.sleep();
  }

}
