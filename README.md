# Rosaria Client

This is a ROS package forked from [rosaria_client](https://github.com/pengtang/rosaria_client), where good demos are available for beginners to learn how to start ROSARIA. Please refer to the link above for detailed information.

This document shows the new features focusing on cooperative multi-robot systems. I perform my research on [Pioneer 3-AT](http://www.mobilerobots.com/researchrobots/p3at.aspx) robots.

Update date: Nov. 30, 2017
by [@hanzheteng](https://github.com/hanzheteng)

# Update Log

Beta V1.0 Most of the time it works fine, sometimes it does not stop(continue going ahead or continue spinning).

Beta V1.1 (Mar. 28) Fix the bug sometimes the robot does not stop Add feature of Stopping the robot first, and then do its functionality

Beta V1.2 (Apr. 5) Add print_state function(could print part of the info right now, need to be enhanced).

Beta V1.3 (Apr. 10) Pushed some small corrections to spin_clockwise, renamed spin_anticlockwise to spin_counterclockwise and added small corrections to that as well. Also added (very) rough draft of interface program.

Beta V1.4 (Apr. 10) Change the spin from 180 to 90 degree, finished the print_state function.

Beta V1.5 (Apr. 14) Add function enable_motors, fix the printing information about the front_motors and rear_motors state.

Beta V1.6 (Apr 16) Added a functional version of the interface

Beta V1.7 (Apr 18) Changed stop key to spacebar in teleop, where the user now hits 'q' to quit.  Also edited interface to prompt user for inputs every time its ready for a selection

Beta V1.8 (Apr. 20) Add launch file for rosaria_client

Beta V1.9 (Apr. 30) Added print_state feature, documentation

Beta V2.0 (Oct. 3, 2017) Forked from [pengtang/rosaria_client](https://github.com/pengtang/rosaria_client)

Beta V2.1 (Oct. 12, 2017) Change default serial port to "ttyS0" instead of "ttyUSB0" in launch file. Fix the bug that robots do not move continuously. (According to [wiki of ROSARIA](http://wiki.ros.org/ROSARIA), there is a WatchDog such that if no cmd_vel messages are received after 600ms, rosaria will stop the robot.) 

Beta V2.2 (Oct. 19, 2017) Added "hand point" control method utilizing feedback linearization. Hand point is a point that 10 inch away from the center. In other words, we regard this robot as a point in the front of itself instead of the center.

Beta V2.3 (Nov. 15, 2017) Added pose control strategy (pose_cmd.cpp); Moved class defination to "Pioneer.h"; Followed standard message format "geometry_msgs::Pose2D".

Beta V2.4 (Nov. 30, 2017) Add a new function: tracking a reference trajectory (circle); change message type of 'vel_hp' from Twist to Vector3 in order to make linear x, y velocity clearly; pack all demos into a directory and modify a new teleop_op fit in Vector3 msg form.

Beta V3.0 (Feb. 5, 2017) Forked from [hanzheteng/rosaria_client](https://github.com/hanzheteng/rosaria_client). Added sonar sensing in 'teleop_hp.cpp'. Pioneer 3-AT will no longer move forward if told to do so if there is an objected too close in front of it.
