cmake_minimum_required(VERSION 3.0.2)
project(singaboat_vrx)

#################################
## Catkin package requirements ##
#################################

# Catkin package requirements
find_package(catkin REQUIRED COMPONENTS
  vrx_gazebo
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
  actionlib_msgs
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
  cfg/teleoperation_config.cfg
  cfg/inverse_kinematics_config.cfg
  cfg/task_manager_config.cfg
  cfg/mission_manager_config.cfg
  cfg/station_keeping_config.cfg
  cfg/wayfinding_config.cfg
  cfg/scene_perception_config.cfg
  cfg/semantic_navigation_config.cfg
  cfg/gymkhana_challenge_config.cfg
  cfg/scan_dock_deliver_config.cfg
)

#################################
## Custom action specification ##
#################################

# Declare custom ROS actions
 add_action_files(
   FILES
   Mission.action
)

# Generate action messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geographic_msgs
  actionlib_msgs
)

#################################
## Catkin runtime dependencies ##
#################################

# Catkin runtime dependencies
catkin_package(CATKIN_DEPENDS
  vrx_gazebo
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
  actionlib_msgs
)

#############
## Install ##
#############

# Install all the script files
catkin_install_python(PROGRAMS
  src/singaboat_teleoperation.py
  src/singaboat_inverse_kinematics.py
  src/singaboat_mission_manager.py
  src/singaboat_task_manager.py
  src/singaboat_station_keeping.py
  src/singaboat_wayfinding.py
  src/singaboat_scene_perception.py
  src/singaboat_semantic_navigation.py
  src/singaboat_gymkhana_challenge.py
  src/singaboat_scan_dock_deliver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install all the script config files
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

# Install all the WAM-V config files
install(DIRECTORY wamv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/wamv)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
