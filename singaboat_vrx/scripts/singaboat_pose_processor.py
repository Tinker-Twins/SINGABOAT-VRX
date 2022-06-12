#!/usr/bin/env python3

import rospy
import math
import pymap3d
from vrx_gazebo.msg import Task
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPath
from singaboat_vrx.msg import Pose3D

################################################################################

class PoseProcessor:
    def __init__(self):
        # Initialize pose processor
        self.cur_pos_x = self.cur_pos_y = self.cur_pos_z = self.cur_roll = self.cur_pitch = self.cur_yaw = None
        self.cmd_pos_x, self.cmd_pos_y, self.cmd_pos_z, self.cmd_roll, self.cmd_pitch, self.cmd_yaw = ([] for i in range(6))
        self.wp_idx         = 0
        self.wp_num_msg     = None
        self.cur_pose_msg   = None
        self.cmd_pose_msg   = None
        self.wp_num_pub     = None
        self.cur_pose_pub   = None
        self.cmd_pose_pub   = None
        self.cmd_topic_name = None
        self.cmd_topic_type = None

    def lla_to_xyz(self, lat, lon, alt): # lat, lon in degrees, alt in meters
        '''
        R = 6378137 # Radius of Earth (in meters)
        f = 0.00335281 # Flattening factor of Earth (WGS84 model)
        cos_lat = math.cos(lat * math.pi / 180)
        sin_lat = math.sin(lat * math.pi / 180)
        cos_lon = math.cos(lon * math.pi / 180)
        sin_lon = math.sin(lon * math.pi / 180)
        c = 1 / math.sqrt(cos_lat * cos_lat + (1 - f) * (1 - f) * sin_lat * sin_lat)
        s = (1 - f) * (1 - f) * c
        x = (R * c + alt) * cos_lat * cos_lon
        y = (R * c + alt) * cos_lat * sin_lon
        z = (R * s + alt) * sin_lat
        return x, y, z # x, y, z in meters (ECEF frame of reference)
        '''
        # Local coordinate origin (Sydney International Regatta Centre, Australia)
        lat0 = -33.7224 # degree North
        lon0 = 150.6712 # degree East
        alt0 = 0 # meters
        enu = pymap3d.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
        x = enu[0]
        y = enu[1]
        z = enu[2]
        return x, y, z # x, y, z in meters (ENU frame of reference)

    def xyzw_to_rpy(self, x, y, z, w): # x, y, z, w in quaternion
        t0 = 2 * (w * x + y * z)
        t1 = 1 - 2 * (x * x + y * y)
        roll = math.atan2(t0, t1) # Roll (about x axis)
        t2 = 2 * (w * y - z * x)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        pitch = math.asin(t2) # Pitch (about y axis)
        t3 = 2 * (w * z + x * y)
        t4 = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(t3, t4) # Yaw (about z axis)
        return roll, pitch, yaw # roll, pitch, yaw in radians

    def cur_pos_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        self.cur_pos_x, self.cur_pos_y, self.cur_pos_z = self.lla_to_xyz(lat, lon, alt)
        # WAM-V frame is +0.85 m offset in local x-axis and -1.3 m offset in local z-axis w.r.t. WAM-V frame GPS frame
        if self.cur_yaw != None:
            self.cur_pos_x += 0.85 * math.cos(self.cur_yaw)
            self.cur_pos_y += 0.85 * math.sin(self.cur_yaw)
            self.cur_pos_z -= 1.3

    def cur_rot_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.cur_roll, self.cur_pitch, self.cur_yaw = self.xyzw_to_rpy(x, y, z, w)

    def wp_idx_callback(self, msg):
        self.wp_idx = msg.data

    def cmd_pose_callback(self, msg):
        if str(msg._type) == 'geographic_msgs/GeoPoseStamped':
            self.wp_num_msg.data = 1
            lat = msg.pose.position.latitude
            lon = msg.pose.position.longitude
            alt = msg.pose.position.altitude
            cmd_pos_x, cmd_pos_y, cmd_pos_z = self.lla_to_xyz(lat, lon, alt)
            self.cmd_pos_x.append(cmd_pos_x)
            self.cmd_pos_y.append(cmd_pos_y)
            self.cmd_pos_z.append(cmd_pos_z)
            x = msg.pose.orientation.x
            y = msg.pose.orientation.y
            z = msg.pose.orientation.z
            w = msg.pose.orientation.w
            cmd_roll, cmd_pitch, cmd_yaw = self.xyzw_to_rpy(x, y, z, w)
            self.cmd_roll.append(cmd_roll)
            self.cmd_pitch.append(cmd_pitch)
            self.cmd_yaw.append(cmd_yaw)
        elif str(msg._type) == 'geographic_msgs/GeoPath':
            for i in range(len(msg.poses)):
                if msg.poses: # Sanity check
                    self.wp_num_msg.data = len(msg.poses)
                    waypoint = msg.poses[i] # Indexing starts from 0
                    lat = waypoint.pose.position.latitude
                    lon = waypoint.pose.position.longitude
                    alt = waypoint.pose.position.altitude
                    cmd_pos_x, cmd_pos_y, cmd_pos_z = self.lla_to_xyz(lat, lon, alt)
                    self.cmd_pos_x.append(cmd_pos_x)
                    self.cmd_pos_y.append(cmd_pos_y)
                    self.cmd_pos_z.append(cmd_pos_z)
                    x = waypoint.pose.orientation.x
                    y = waypoint.pose.orientation.y
                    z = waypoint.pose.orientation.z
                    w = waypoint.pose.orientation.w
                    cmd_roll, cmd_pitch, cmd_yaw = self.xyzw_to_rpy(x, y, z, w)
                    self.cmd_roll.append(cmd_roll)
                    self.cmd_pitch.append(cmd_pitch)
                    self.cmd_yaw.append(cmd_yaw)

    def pose_processor(self):
        # Current pose
        if not [x for x in (self.cur_pos_x, self.cur_pos_y, self.cur_pos_z, self.cur_roll, self.cur_pitch, self.cur_yaw) if x is None]:
            self.cur_pose_msg.pos_x = self.cur_pos_x
            self.cur_pose_msg.pos_y = self.cur_pos_y
            #self.cur_pose_msg.pos_z = self.cur_pos_z
            #self.cur_pose_msg.rot_x = self.cur_roll
            #self.cur_pose_msg.rot_y = self.cur_pitch
            self.cur_pose_msg.rot_z = self.cur_yaw
            print("Cur_Pose: %.4f m %.4f m %.4f rad" % (self.cur_pose_msg.pos_x, self.cur_pose_msg.pos_y, self.cur_pose_msg.rot_z))
            self.cur_pose_pub.publish(self.cur_pose_msg)
        # Commanded pose
        if self.cmd_pos_x and self.cmd_pos_y and self.cmd_pos_z and self.cmd_roll and self.cmd_pitch and self.cmd_yaw:
            self.cmd_pose_msg.pos_x = self.cmd_pos_x[self.wp_idx]
            self.cmd_pose_msg.pos_y = self.cmd_pos_y[self.wp_idx]
            #self.cmd_pose_msg.pos_z = self.cmd_pos_z[self.wp_idx]
            #self.cmd_pose_msg.rot_x = self.cmd_roll[self.wp_idx]
            #self.cmd_pose_msg.rot_y = self.cmd_pitch[self.wp_idx]
            self.cmd_pose_msg.rot_z = self.cmd_yaw[self.wp_idx]
            print("WP_Index: %d" % (self.wp_idx))
            print("Cmd_Pose: %.4f m %.4f m %.4f rad" % (self.cmd_pose_msg.pos_x, self.cmd_pose_msg.pos_y, self.cmd_pose_msg.rot_z))
            self.cmd_pose_pub.publish(self.cmd_pose_msg)
        # Number of waypoints
        self.wp_num_pub.publish(self.wp_num_msg)

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_pose_processor', anonymous = True)

    # PoseProcessor class instance
    pose_processor_node = PoseProcessor()

    # Task specific topic selection logic
    task_msg = rospy.wait_for_message('/vrx/task/info', Task)
    if task_msg.name == 'station_keeping':
        pose_processor_node.cmd_topic_name = '/vrx/station_keeping/goal'
        pose_processor_node.cmd_topic_type = GeoPoseStamped
    elif task_msg.name == 'wayfinding':
        pose_processor_node.cmd_topic_name = '/vrx/wayfinding/waypoints'
        pose_processor_node.cmd_topic_type = GeoPath
    elif task_msg.name == 'wildlife':
        pose_processor_node.cmd_topic_name = '/vrx/wildlife/animals/poses'
        pose_processor_node.cmd_topic_type = GeoPath

    # Messages
    pose_processor_node.wp_num_msg   = Int32()
    pose_processor_node.cur_pose_msg = Pose3D()
    pose_processor_node.cmd_pose_msg = Pose3D()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, pose_processor_node.cur_pos_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, pose_processor_node.cur_rot_callback)
    rospy.Subscriber('/vrx/wp_idx', Int32, pose_processor_node.wp_idx_callback)
    rospy.Subscriber(pose_processor_node.cmd_topic_name, pose_processor_node.cmd_topic_type, pose_processor_node.cmd_pose_callback)

    # Publishers
    pose_processor_node.wp_num_pub   = rospy.Publisher('/vrx/wp_num', Int32, queue_size = 10)
    pose_processor_node.cur_pose_pub = rospy.Publisher('/wamv/cur_pose', Pose3D, queue_size = 10)
    pose_processor_node.cmd_pose_pub = rospy.Publisher('/wamv/cmd_pose', Pose3D, queue_size = 10)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            pose_processor_node.pose_processor()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
