#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg
from vrx_gazebo.msg import Task

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_task_manager', anonymous = True)

    print()
    print('==========================================================')
    print('          Virtual RobotX (VRX) Competition 2022           ')
    print('==========================================================')
    print('TEAM NAME : SINGABOAT-VRX')
    print('INSTITUTE : Nanyang Technological University, Singapore')
    print('MEMBERS   : Tanmay Samak, Chinmay Samak and Chern Peng Lee')
    print('ADVISOR   : Dr. Ming Xie')
    print('==========================================================')
    print()

    # Get path to singaboat_vrx ROS package
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('singaboat_vrx')

    # Launch task-specific launch file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print('\nWAITING FOR A VALID VRX TASK...')
    task_msg = rospy.wait_for_message('/vrx/task/info', Task)
    if task_msg.name == 'station_keeping':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + '/launch/vrx_station_keeping.launch'])
        print()
        print('LAUNCHING SOLUTION FOR STATION KEEPING TASK...')
        print()
        launch.start()
    elif task_msg.name == 'wayfinding':
        launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_path + '/launch/vrx_wayfinding.launch'])
        print()
        print('LAUNCHING SOLUTION FOR WAYFINDING TASK...')
        print()
        launch.start()
    else:
        print()
        print('CURRENTLY NO SOLUTION IS AVAILABLE FOR THIS TASK!')
        print()
        rospy.signal_shutdown('CURRENTLY NO SOLUTION IS AVAILABLE FOR THIS TASK!')

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
