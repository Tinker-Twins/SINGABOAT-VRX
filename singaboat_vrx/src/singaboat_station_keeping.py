#!/usr/bin/env python3

import math
import numpy
import rospy
import tf
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import StationKeepingConfig
from singaboat_vrx.control_module import PIDController
from singaboat_vrx.common_utilities import gps_to_enu, quaternion_to_euler, euler_to_quaternion, normalize_angle

################################################################################

class StationKeeping:
    def __init__(self):
        # Initialize station-keeping
        self.cur_pos      = None # Current 2D position (x, y)
        self.cur_rot      = None # Current 1D orientation (yaw)
        self.cur_position = None # Current 3D position (x, y, z)
        self.cur_rotation = None # Current 3D orientation (roll, pitch, yaw)
        self.cmd_pos      = None # Commanded 2D position (x, y)
        self.cmd_rot      = None # Commanded 1D orientation (yaw)
        self.cmd_position = None # Commanded 3D position (x, y, z)
        self.cmd_rotation = None # Commanded 3D orientation (roll, pitch, yaw)
        self.time         = None # Current timestamp
        self.config       = {} # Station-keeping configuration
        # ROS infrastructure
        self.tf_broadcaster = None
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

    def gps_callback(self, msg):
        if self.cur_rot is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        pos_x += self.gps_offset * math.cos(self.cur_rot[2])
        pos_y += self.gps_offset * math.sin(self.cur_rot[2])
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])
        self.cur_rotation = euler_to_quaternion(self.cur_rot[0], self.cur_rot[1], self.cur_rot[2])

    def imu_callback(self, msg):
        self.cur_rot = quaternion_to_euler(msg.orientation)

    def goal_callback(self, msg):
        # Transform goal pose from GPS to ENU frame of reference
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        self.cmd_rot = quaternion_to_euler(msg.pose.orientation)
        self.cmd_pos = numpy.array([pos_x, pos_y])
        self.cmd_position = numpy.array([pos_x, pos_y, pos_z])
        self.cmd_rotation = euler_to_quaternion(self.cmd_rot[0], self.cmd_rot[1], self.cmd_rot[2])

    def station_keeping(self):
        # Broadcast transform of `wamv/base_link` frame w.r.t. `world` frame
        self.tf_broadcaster.sendTransform((self.cur_position[0], self.cur_position[1], self.cur_position[2]), (self.cur_rotation[0], self.cur_rotation[1], self.cur_rotation[2], self.cur_rotation[3]), rospy.Time.now(), 'wamv/base_link', 'world')
        # Broadcast transform of `goal` frame w.r.t. `world` frame
        self.tf_broadcaster.sendTransform((self.cmd_position[0], self.cmd_position[1], self.cmd_position[2]), (self.cmd_rotation[0], self.cmd_rotation[1], self.cmd_rotation[2], self.cmd_rotation[3]), rospy.Time.now(), 'goal', 'world')
        # Control algorithm
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.cur_pos, self.cur_rot, self.cmd_pos, self.cmd_rot, self.time) if x is None]:
            err_pos = self.cmd_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
            if numpy.linalg.norm(err_pos) > self.goal_tol: # (Euclidean distance to goal as L2 norm)
                # If far from goal, head directly towards the goal by controlling Vx & Wz
                lin_vel_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for Vx
                if lin_vel_x > self.v_limit: # Clamp linear velocity along X-axis
                    lin_vel_x = self.v_limit
                lin_vel_y = 0.0
                err_rot = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot[2]) # Error in orientation
                ang_vel_z = self.pid_g2g.control(err_rot, self.time) # PID controller for Wz
            else:
                # If near goal, perform fine adjustments in Vx, Vy & Wz for station-keeping
                rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot[2]) # G2G rotation transformation
                err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)]) # Error in position (in local frame)
                lin_vel_x = self.pid_sk_vx.control(err_pos[0], self.time) # PID controller for Vx
                lin_vel_y = self.pid_sk_vy.control(err_pos[1], self.time) # PID controller for Vy
                err_rot = normalize_angle(self.cmd_rot[2] - self.cur_rot[2]) # Error in orientation
                ang_vel_z = self.pid_sk_wz.control(err_rot, self.time) # PID controller for Wz
            print("Station-Keeping Coordinates: {:.4} m, {:.4} m, {:.4} rad".format(self.cmd_pos[0], self.cmd_pos[1], self.cmd_rot[2]))
            print()
            if self.debug:
                print("Error Pos X: {:.4} m".format(err_pos[0]))
                print("Error Pos Y: {:.4} m".format(err_pos[1]))
                print("Error Rot Z: {:.4} rad".format(err_rot))
                print()
            # Generate and publish `cmd_vel` message
            self.cmd_vel_msg.linear.x = lin_vel_x
            self.cmd_vel_msg.linear.y = lin_vel_y
            self.cmd_vel_msg.angular.z = ang_vel_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.pid_g2g    = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        self.pid_sk_vx  = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        self.pid_sk_vy  = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        self.pid_sk_wz  = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        self.goal_tol   = config['goal_tol'] # Goal tolerance dead-band of go-to-goal PID controller
        self.v_const    = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit    = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.debug      = config['debug'] # Flag to enable/disable debug messages
        self.config     = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_station_keeping')

    # StationKeeping class instance
    station_keeping_node = StationKeeping()

    # Dynamic reconfigure server
    station_keeping_node.dyn_reconf_srv = Server(StationKeepingConfig, station_keeping_node.config_callback)

    # Transform broadcaster
    station_keeping_node.tf_broadcaster = tf.TransformBroadcaster()

    # Message
    station_keeping_node.cmd_vel_msg  = Twist()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, station_keeping_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, station_keeping_node.imu_callback)
    rospy.Subscriber('/vrx/station_keeping/goal', GeoPoseStamped, station_keeping_node.goal_callback)

    # Publisher
    station_keeping_node.cmd_vel_pub = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/vrx/station_keeping/goal', GeoPoseStamped)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            station_keeping_node.station_keeping()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
