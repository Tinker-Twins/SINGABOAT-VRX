#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu, Image, PointCloud2
from geographic_msgs.msg import GeoPoseStamped
from dynamic_reconfigure.server import Server
from vrx_gazebo.msg import Task
from singaboat_vrx.cfg import ScenePerceptionConfig
from singaboat_vrx.perception_module import BuoyDetector, GateDetector

################################################################################

'''
`BuoyDetector` and `GateDetector` classes are defined in singaboat_vrx.perception_module
'''

################################################################################
if __name__ == '__main__':
    rospy.init_node('singaboat_scene_perception')

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/wamv/sensors/cameras/camera/image_raw', Image)
    rospy.wait_for_message('/wamv/sensors/lidars/lidar/points', PointCloud2)

    task_msg = rospy.wait_for_message('/vrx/task/info', Task)

    if task_msg.name == 'perception':
        # BuoyDetector class instance
        scene_perception_node = BuoyDetector()
        # Dynamic reconfigure server
        scene_perception_node.dyn_reconf_srv = Server(ScenePerceptionConfig, scene_perception_node.config_callback)
        # Messages
        scene_perception_node.cam_viz_msg = Image()
        scene_perception_node.object_msg  = GeoPoseStamped()
        # Subscribers
        rospy.Subscriber('/vrx/task/info', Task, scene_perception_node.task_callback)
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, scene_perception_node.gps_callback)
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, scene_perception_node.imu_callback)
        rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, scene_perception_node.camera_callback)
        rospy.Subscriber('/wamv/sensors/lidars/lidar/points', PointCloud2, scene_perception_node.lidar_callback)
        # Publishers
        scene_perception_node.cam_viz_pub = rospy.Publisher('/rviz/cam_viz', Image, queue_size=10)
        scene_perception_node.object_pub  = rospy.Publisher('/vrx/perception/landmark', GeoPoseStamped, queue_size=10)
        # Recursive loop
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    elif task_msg.name == 'gymkhana':
        # GateDetector class instance
        gate_detector = GateDetector()
        # Dynamic reconfigure server
        gate_detector.buoy_detector.dyn_reconf_srv = Server(ScenePerceptionConfig, gate_detector.config_callback)
        # Messages
        gate_detector.buoy_detector.cam_viz_msg  = Image()
        gate_detector.buoy_detector.object_msg   = GeoPoseStamped()
        gate_detector.buoy_detector.obstacle_msg = Float64MultiArray()
        # Subscribers
        rospy.Subscriber('/vrx/task/info', Task, gate_detector.buoy_detector.task_callback)
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, gate_detector.buoy_detector.gps_callback)
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, gate_detector.buoy_detector.imu_callback)
        rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, gate_detector.buoy_detector.camera_callback)
        rospy.Subscriber('/wamv/sensors/lidars/lidar/points', PointCloud2, gate_detector.buoy_detector.lidar_callback)
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, gate_detector.gps_callback)
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, gate_detector.imu_callback)
        # Publishers
        gate_detector.buoy_detector.cam_viz_pub  = rospy.Publisher('/rviz/cam_viz', Image, queue_size=10)
        gate_detector.buoy_detector.object_pub   = rospy.Publisher('/wamv/detected_objects', GeoPoseStamped, queue_size=10)
        gate_detector.buoy_detector.obstacle_pub = rospy.Publisher('/wamv/detected_obstacles', Float64MultiArray, queue_size=10)
        # ROS rate
        rate = rospy.Rate(20)
        # Recursive loop
        try:
            while not rospy.is_shutdown():
                gate_detector.detected_objects = gate_detector.buoy_detector.detected_objects
                if gate_detector.asv_pose is not None:
                    gate_detector.detect_entrance()
                    if gate_detector.entrance_detected and not gate_detector.entrance_reported:
                        gate_detector.buoy_detector.report_detected_objects(gate_detector.entrance_position[0], gate_detector.entrance_position[1], "Entrance",gate_detector.entrance_orientation)
                        gate_detector.entrance_reported = True
                    if gate_detector.entrance_detected:
                        if not gate_detector.exit_detected:
                            gate_detector.detect_gate()
                            if gate_detector.gate_1_detected and not gate_detector.gate_1_reported:
                                gate_detector.buoy_detector.report_detected_objects(gate_detector.gate_1_position[0], gate_detector.gate_1_position[1], "Gate 1",gate_detector.gate_1_orientation)
                                gate_detector.gate_1_reported = True
                            if gate_detector.gate_2_detected and not gate_detector.gate_2_reported:
                                gate_detector.buoy_detector.report_detected_objects(gate_detector.gate_2_position[0], gate_detector.gate_2_position[1], "Gate 2",gate_detector.gate_2_orientation)
                                gate_detector.gate_2_reported = True
                            if gate_detector.gate_3_detected and not gate_detector.gate_3_reported:
                                gate_detector.buoy_detector.report_detected_objects(gate_detector.gate_3_position[0], gate_detector.gate_3_position[1], "Gate 3",gate_detector.gate_3_orientation)
                                gate_detector.gate_3_reported = True
                        gate_detector.detect_exit()
                        if gate_detector.exit_detected and not gate_detector.exit_reported:
                            gate_detector.buoy_detector.report_detected_objects(gate_detector.exit_position[0], gate_detector.exit_position[1], "Exit",gate_detector.exit_orientation)
                            gate_detector.exit_reported = True
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
