#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class TeleopRosAria
{
  public:
    TeleopRosAria();
    void keyLoop();
    void isFarEnough(const sensor_msgs::PointCloud&);
  private:
    ros::NodeHandle nh_;
    double linear_x, linear_y, x_scale_, y_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber sub_sonar;
    bool far3 = true;
    bool far4 = true;
    sensor_msgs::PointCloud sonar_cmd;
};
TeleopRosAria::TeleopRosAria():
  linear_x(0),
  linear_y(0),
  x_scale_(2.0),
  y_scale_(2.0)
{
  nh_.param("scale_linear_x", x_scale_, x_scale_);
  nh_.param("scale_linear_y", y_scale_, y_scale_);
  vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("RosAria/cmd_vel_hp", 1);
  sub_sonar = nh_.subscribe("RosAria/sonar", 1, &TeleopRosAria::isFarEnough, this);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hp");
  TeleopRosAria teleop_RosAria;
  signal(SIGINT,quit);
  teleop_RosAria.keyLoop();
  return(0);
}
void TeleopRosAria::keyLoop()
{
  char c;
  bool dirty=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  for(;;)
  {
    ros::spinOnce();
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }
    linear_x=linear_y=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
    	  linear_y = 0.1;
    	  linear_x = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
    	  linear_y = -0.1;
    	  linear_x = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_U:
	      if(far3 && far4)
        {
          ROS_DEBUG("UP");
          linear_x = 0.1;
          linear_y = 0;
          dirty = true;
        }

        else
        {
          ROS_DEBUG("UP");
          linear_x = 0;
          linear_y = 0;
          dirty = true;
        }
        break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
    	  linear_x = -0.1;
    	  linear_y = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
    	  linear_x = 0;
    	  linear_y = 0;
    	  dirty = true;
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        ROS_INFO_STREAM("You quit the teleop successfully");
        return;
        break;
  	}
    geometry_msgs::Vector3 vel;
    vel.x = x_scale_*linear_x;
    vel.y = y_scale_*linear_y;
    vel.z = 0;
    if(dirty == true)
  	{
  	  vel_pub_.publish(vel);
  	  dirty=false;
    }
  }
  return;
}

void TeleopRosAria::isFarEnough(const sensor_msgs::PointCloud& msg)
{
  this->sonar_cmd = msg;
  if (this->sonar_cmd.points[3].x < 0.8)
  {
    far3 = false;
  }
  else
  {
    far3 = true;
  }
  if (this->sonar_cmd.points[4].x < 0.8)
  {
    far4 = false;
  }
  else
  {
    far4 = true;
  }
}
