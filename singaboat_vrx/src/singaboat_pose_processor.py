#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Pose # !!! NOT USED AS PER THE STANDARDS !!!
from dynamic_reconfigure.server import Server
from vrx_gazebo.msg import Task
from singaboat_vrx.cfg import PoseProcessorConfig
from singaboat_vrx.common_utilities import gps_to_enu, quaternion_to_euler

################################################################################

class PoseProcessor:
    def __init__(self):
        # Initialize pose processor
        self.cur_pos_x = self.cur_pos_y = self.cur_yaw = None # Current state
        self.cmd_pos_x, self.cmd_pos_y, self.cmd_yaw = ([] for i in range(3)) # Commanded state
        self.wp_index       = 0 # Waypoint index (starts from 0)
        self.cmd_topic_name = None # Topic name for commanded state
        self.cmd_topic_type = None # Topic type for commanded state
        self.config         = {} # Pose processor configuration
        # ROS infrastructure
        self.wp_count_msg   = None
        self.cur_pose_msg   = None
        self.cmd_pose_msg   = None
        self.wp_count_pub   = None
        self.cur_pose_pub   = None
        self.cmd_pose_pub   = None
        self.dyn_reconf_srv = None

    def cur_pos_callback(self, msg):
        if self.cur_yaw is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        self.cur_pos_x, self.cur_pos_y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        self.cur_pos_x += self.gps_offset * math.cos(self.cur_yaw)
        self.cur_pos_y += self.gps_offset * math.sin(self.cur_yaw)

    def cur_rot_callback(self, msg):
        self.cur_yaw = quaternion_to_euler(msg.orientation)[2]

    def wp_index_callback(self, msg):
        self.wp_index = msg.data

    def cmd_pose_callback(self, msg):
        if str(msg._type) == 'geographic_msgs/GeoPoseStamped':
            self.wp_count_msg.data = 1
            lat = msg.pose.position.latitude
            lon = msg.pose.position.longitude
            cmd_pos_x, cmd_pos_y, _ = gps_to_enu(lat, lon)
            self.cmd_pos_x.append(cmd_pos_x)
            self.cmd_pos_y.append(cmd_pos_y)
            cmd_yaw = quaternion_to_euler(msg.pose.orientation)[2]
            self.cmd_yaw.append(cmd_yaw)
        elif str(msg._type) == 'geographic_msgs/GeoPath':
            for i in range(len(msg.poses)):
                if msg.poses: # Sanity check
                    self.wp_count_msg.data = len(msg.poses)
                    waypoint = msg.poses[i] # Indexing starts from 0
                    lat = waypoint.pose.position.latitude
                    lon = waypoint.pose.position.longitude
                    cmd_pos_x, cmd_pos_y, _ = gps_to_enu(lat, lon)
                    self.cmd_pos_x.append(cmd_pos_x)
                    self.cmd_pos_y.append(cmd_pos_y)
                    cmd_yaw = quaternion_to_euler(waypoint.pose.orientation)[2]
                    self.cmd_yaw.append(cmd_yaw)

    def pose_processor(self):
        # Current pose
        if not [x for x in (self.cur_pos_x, self.cur_pos_y, self.cur_yaw) if x is None]:
            self.cur_pose_msg.position.x = self.cur_pos_x
            self.cur_pose_msg.position.y = self.cur_pos_y
            self.cur_pose_msg.orientation.z = self.cur_yaw
            print("Cur_Pose: %.4f m %.4f m %.4f rad" % (self.cur_pose_msg.position.x, self.cur_pose_msg.position.y, self.cur_pose_msg.orientation.z))
            self.cur_pose_pub.publish(self.cur_pose_msg)
        # Commanded pose
        if self.cmd_pos_x and self.cmd_pos_y and self.cmd_yaw:
            self.cmd_pose_msg.position.x = self.cmd_pos_x[self.wp_index]
            self.cmd_pose_msg.position.y = self.cmd_pos_y[self.wp_index]
            self.cmd_pose_msg.orientation.z = self.cmd_yaw[self.wp_index]
            print("WP_Index: %d" % (self.wp_index))
            print("Cmd_Pose: %.4f m %.4f m %.4f rad" % (self.cmd_pose_msg.position.x, self.cmd_pose_msg.position.y, self.cmd_pose_msg.orientation.z))
            self.cmd_pose_pub.publish(self.cmd_pose_msg)
        # Waypoint count
        print("WP_Count: %d" % (self.wp_count_msg.data))
        self.wp_count_pub.publish(self.wp_count_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.gps_offset = config['gps_offset']
        self.config     = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_pose_processor', anonymous = True)

    # PoseProcessor class instance
    pose_processor_node = PoseProcessor()

    # Dynamic reconfigure server
    pose_processor_node.dyn_reconf_srv = Server(PoseProcessorConfig, pose_processor_node.config_callback)

    # Task specific topic selection logic
    task_msg = rospy.wait_for_message('/vrx/task/info', Task)
    if task_msg.name == 'station_keeping':
        pose_processor_node.cmd_topic_name = '/vrx/station_keeping/goal'
        pose_processor_node.cmd_topic_type = GeoPoseStamped
    elif task_msg.name == 'wayfinding':
        pose_processor_node.cmd_topic_name = '/vrx/wayfinding/waypoints'
        pose_processor_node.cmd_topic_type = GeoPath

    # Messages
    pose_processor_node.wp_count_msg = Int32()
    pose_processor_node.cur_pose_msg = Pose()
    pose_processor_node.cmd_pose_msg = Pose()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, pose_processor_node.cur_pos_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, pose_processor_node.cur_rot_callback)
    rospy.Subscriber('/vrx/wp_index', Int32, pose_processor_node.wp_index_callback)
    rospy.Subscriber(pose_processor_node.cmd_topic_name, pose_processor_node.cmd_topic_type, pose_processor_node.cmd_pose_callback)

    # Publishers
    pose_processor_node.wp_count_pub = rospy.Publisher('/vrx/wp_count', Int32, queue_size = 10)
    pose_processor_node.cur_pose_pub = rospy.Publisher('/wamv/cur_pose', Pose, queue_size = 10)
    pose_processor_node.cmd_pose_pub = rospy.Publisher('/wamv/cmd_pose', Pose, queue_size = 10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            pose_processor_node.pose_processor()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
