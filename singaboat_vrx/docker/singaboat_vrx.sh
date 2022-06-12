#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source /home/$USER/vrx_ws/devel/setup.bash

# Launch SingaBoat-VRX solution(s) for VRX competition
roslaunch singaboat_vrx singaboat_vrx.launch
