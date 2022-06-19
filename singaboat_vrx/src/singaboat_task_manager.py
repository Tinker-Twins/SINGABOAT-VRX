#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg
from dynamic_reconfigure.server import Server
from vrx_gazebo.msg import Task
from singaboat_vrx.cfg import TaskManagerConfig

################################################################################

info = """

============================================================
      SINGABOAT-VRX | Virtual RobotX (VRX) Competition
============================================================
COMPETITION : Virtual RobotX (VRX) Competition 2022
TEAM NAME   : SINGABOAT-VRX
INSTITUTE   : Nanyang Technological University, Singapore
MEMBERS     : Tanmay Samak, Chinmay Samak and Chern Peng Lee
ADVISOR     : Dr. Ming Xie
============================================================

"""

def config_callback(config, level):
    return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_task_manager', anonymous = True)

    # Dynamic reconfigure server
    dyn_reconf_srv = Server(TaskManagerConfig, config_callback)

    # Print team details
    if dyn_reconf_srv.config['debug']:
        print(info)

    # Get path to singaboat_vrx ROS package
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('singaboat_vrx')

    # Launch task-specific launch file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    if dyn_reconf_srv.config['debug']:
        print('\nWAITING FOR A VALID VRX TASK...')
    task_msg = rospy.wait_for_message('/vrx/task/info', Task)
    if task_msg.name == 'station_keeping':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_1_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: STATION-KEEPING')
            print()
            print('LAUNCHING SOLUTION FOR STATION-KEEPING TASK...')
            print()
        launch.start()
    elif task_msg.name == 'wayfinding':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_2_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: WAYFINDING')
            print()
            print('LAUNCHING SOLUTION FOR WAYFINDING TASK...')
            print()
        launch.start()
    elif task_msg.name == 'perception':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_3_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: SCENE PERCEPTION')
            print()
            print('LAUNCHING SOLUTION FOR SCENE PERCEPTION TASK...')
            print()
        launch.start()
    elif task_msg.name == 'wildlife':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_4_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: SEMANTIC NAVIGATION')
            print()
            print('LAUNCHING SOLUTION FOR SEMANTIC NAVIGATION TASK...')
            print()
        launch.start()
    elif task_msg.name == 'gymkhana':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_5_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: GYMKHANA CHALLENGE')
            print()
            print('LAUNCHING SOLUTION FOR GYMKHANA CHALLENGE TASK...')
            print()
        launch.start()
    elif task_msg.name == 'scan_dock_deliver':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + dyn_reconf_srv.config['task_6_launch_path']])
        if dyn_reconf_srv.config['debug']:
            print()
            print('DETECTED VRX TASK: SCAN-DOCK-DELIVER')
            print()
            print('LAUNCHING SOLUTION FOR SCAN-DOCK-DELIVER TASK...')
            print()
        launch.start()
    else:
        if dyn_reconf_srv.config['debug']:
            print()
            print('NO VALID VRX TASK DETECTED!')
            print()
        rospy.signal_shutdown('NO VALID VRX TASK DETECTED!')

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
