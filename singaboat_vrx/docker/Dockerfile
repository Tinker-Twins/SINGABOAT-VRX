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
 && pip3 install --no-cache-dir numpy pymap3d

# Set working directory.
WORKDIR /home/$USER

# Install VRX packages.
RUN mkdir -p vrx_ws/src/
COPY ./ vrx_ws/src/
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd vrx_ws && catkin_make' \
 && /bin/bash -c 'echo "source /home/$USER/vrx_ws/devel/setup.bash" >> ~/.bashrc' \
 && /bin/bash -c 'source ~/.bashrc'

# Set entrypoint.
ENTRYPOINT ["vrx_ws/src/vrx/singaboat_vrx/docker/singaboat_vrx.sh"]
