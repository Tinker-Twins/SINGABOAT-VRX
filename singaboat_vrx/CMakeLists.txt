cmake_minimum_required(VERSION 3.0.2)
project(singaboat_vrx)

#################################
## Catkin package requirements ##
#################################

# Catkin package requirements
find_package(catkin REQUIRED COMPONENTS
  vrx_gazebo
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  geographic_msgs
  tf
  xacro
  dynamic_reconfigure
  message_generation
)

########################################
## Install Python scripts and modules ##
########################################

# Setup to install Python scripts and modules
catkin_python_setup()

#################################
## Dynamic reconfigure options ##
#################################

# Dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/ik_solver_config.cfg
  cfg/station_keeping_config.cfg
  cfg/wayfinding_config.cfg
)

################################
## Custom message definitions ##
################################

# Declare custom ROS messages
add_message_files(
  FILES
  Pose3D.msg
  Pose3DStamped.msg
  Pose6D.msg
  Pose6DStamped.msg
)

# Generate declared messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

#################################
## Catkin runtime dependencies ##
#################################

# Catkin runtime dependencies
catkin_package(CATKIN_DEPENDS
  vrx_gazebo
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  geographic_msgs
  tf
  xacro
  dynamic_reconfigure
  message_runtime
)

#############
## Install ##
#############

# Install all the script files
catkin_install_python(PROGRAMS
  scripts/singaboat_teleop_key.py
  scripts/singaboat_pose_processor.py
  scripts/singaboat_ik_solver.py
  scripts/singaboat_task_manager.py
  scripts/singaboat_station_keeping.py
  scripts/singaboat_wayfinding.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install all the cfg files
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
