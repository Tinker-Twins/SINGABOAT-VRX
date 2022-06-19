#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source /home/$USER/VRX_Workspace/devel/setup.bash

# Launch VNC server
x11vnc -forever -usepw -create & /bin/bash
