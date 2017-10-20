#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
// sub /cmd_vel_hp	
// pub /cmd_vel
// launch  map -> 


class Pioneer
{
  private:
    ros::Subscriber sub_hp;
    ros::Subscriber sub_pose;
    ros::Publisher pub;

  public:
    //nav_msgs::Odometry theta;
    double theta;
    geometry_msgs::Twist vel;
    geometry_msgs::Twist vel_hp;
    const float len = 0.25;  //by meter: 10in = 0.25m

    Pioneer(ros::NodeHandle& nh)
    {
        sub_hp = nh.subscribe("RosAria/cmd_vel_hp", 1, &Pioneer::handPointCallBack, this);
        sub_pose = nh.subscribe("RosAria/pose", 1, &Pioneer::poseCallBack, this);
        pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    }

    ~Pioneer()
    {

    }

    void poseCallBack(const nav_msgs::Odometry& msg) 
    {
      tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      ROS_INFO_STREAM("roll="<<roll<<"; pitch="<<pitch<<"; yaw="<<yaw<<";\n");
      this->theta = yaw;  //yaw from -3.14 to 3.14, left side is the positive number
    }

    void handPointCallBack(const geometry_msgs::Twist& msg)
    {
      this->vel_hp = msg;
      this->pubVel();
    }

    void pubVel()
    {
      float x, y, v, w;
      x = vel_hp.linear.x;
      y = vel_hp.angular.z;
/*

 v   1    L*cos0  L*sin0     x
   = - *                  * 
 w   L    -sin0    cos0      y

*/  
      v = x*cos(theta) + y*sin(theta);
      w = ( x*(-sin(theta)) + y*cos(theta) )/len; 

      ROS_INFO_STREAM("x="<<x<<"; y="<<y<<"; v="<<v<<"; w="<<w<<"; theta="<<theta<<";\n");
      vel.linear.x = v;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = w;
      pub.publish(this->vel);
    }

};




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
