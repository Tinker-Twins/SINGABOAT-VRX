# Pull ROS Noetic base image.
FROM ros:noetic-ros-base

# Insall package dependencies.
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections \
 && apt update \
 && apt install -y \
        nano \
        python3-pip \
        ros-noetic-xacro \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-plugins \
        ros-noetic-hector-gazebo-plugins \
        ros-noetic-joy \
        ros-noetic-joy-teleop \
        ros-noetic-key-teleop \
        ros-noetic-robot-localization \
        ros-noetic-robot-state-publisher \
        ros-noetic-rviz \
        ros-noetic-teleop-tools \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-velodyne-simulator \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
 && apt clean \
 && pip3 install --no-cache-dir --upgrade pip \
 && pip3 install --no-cache-dir numpy==1.22.3 scipy==1.8.0 pymap3d==2.7.2 open3d==0.13.0 dubins==1.0.1

# Set working directory.
WORKDIR /home/$USER

# Install VRX packages.
RUN mkdir -p VRX_Workspace/src/
COPY ./ VRX_Workspace/src/
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd VRX_Workspace && catkin_make' \
 && /bin/bash -c 'echo "source /home/$USER/VRX_Workspace/devel/setup.bash" >> ~/.bashrc' \
 && /bin/bash -c 'source ~/.bashrc'

# Set entrypoint.
ENTRYPOINT ["VRX_Workspace/src/SINGABOAT-VRX/singaboat_vrx/scripts/singaboat_vrx.sh"]
