# Rosaria Client

This is a ROS package that I ([@hanzheteng](https://github.com/hanzheteng)) implement some control algorithms for [Pioneer 3-AT](http://www.mobilerobots.com/researchrobots/p3at.aspx) robots.

This package is forked from [rosaria_client](https://github.com/pengtang/rosaria_client), which has a good demo client program and some tutorials. Please refer to the link above for detailed information. This document only focuses on the new features.

Update date: Oct. 3, 2017

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
