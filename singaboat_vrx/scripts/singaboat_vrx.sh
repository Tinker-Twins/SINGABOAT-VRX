#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source /home/$USER/VRX_Workspace/devel/setup.bash

# Launch SINGABOAT-VRX solution(s) for VRX Competition
roslaunch singaboat_vrx singaboat_vrx.launch
