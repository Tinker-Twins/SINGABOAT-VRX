#!/usr/bin/env python3

import math
import numpy
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Twist # !!! NOT USED AS PER THE STANDARDS !!!
from singaboat_vrx.cfg import WayfindingConfig
from singaboat_vrx.control_module import PIDController
from singaboat_vrx.common_utilities import normalize_angle

################################################################################

class Wayfinding:
    def __init__(self):
        # Initialize wayfinding
        self.cmd_pos  = None # Desired 2D position (x, y)
        self.cmd_rot  = None # Desired 1D orientation (yaw)
        self.cur_pos  = None # Current 2D position (x, y)
        self.cur_rot  = None # Current 1D orientation (yaw)
        self.wp_count = None # Waypoint count (starts from 1)
        self.wp_index = 0 # Waypoint index (starts from 0)
        self.time     = None # Current timestamp
        self.config   = {} # Wayfinding configuration
        # ROS infrastructure
        self.wp_index_msg   = None
        self.cmd_vel_msg    = None
        self.wp_index_pub   = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

    def wp_count_callback(self, msg):
        self.wp_count = msg.data

    def cmd_pose_callback(self, msg):
        pos_x = msg.position.x
        pos_y = msg.position.y
        rot_z = msg.orientation.z
        self.cmd_pos = numpy.array([pos_x, pos_y])
        self.cmd_rot = rot_z

    def cur_pose_callback(self, msg):
        pos_x = msg.position.x
        pos_y = msg.position.y
        rot_z = msg.orientation.z
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_rot = rot_z

    def wayfinding(self):
        # Proceed to current waypoint until it is actually reached
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.wp_count, self.cmd_pos, self.cmd_rot, self.cur_pos, self.cur_rot, self.time) if x is None]:
            err_pos = self.cmd_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
            if numpy.linalg.norm(err_pos) > self.goal_tol: # (Euclidean distance to goal as L2 norm)
                # If far from goal, head directly towards the goal by controlling Vx & Wz
                lin_vel_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for Vx
                if lin_vel_x > self.v_limit: # Clamp linear velocity along X-axis
                    lin_vel_x = self.v_limit
                lin_vel_y = 0.0
                err_rot = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot) # Error in orientation
                ang_vel_z = self.pid_g2g.control(err_rot, self.time) # PID controller for Wz
            else:
                # If near goal, perform fine adjustments in Vx, Vy & Wz for station-keeping
                rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - self.cur_rot) # G2G rotation transformation
                err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)]) # Error in position (in local frame)
                lin_vel_x = self.pid_sk_vx.control(err_pos[0], self.time) # PID controller for Vx
                lin_vel_y = self.pid_sk_vy.control(err_pos[1], self.time) # PID controller for Vy
                err_rot = normalize_angle(self.cmd_rot - self.cur_rot) # Error in orientation
                ang_vel_z = self.pid_sk_wz.control(err_rot, self.time) # PID controller for Wz
            print("WP_Count: %d" % (self.wp_count))
            print("WP_Index: %d" % (self.wp_index))
            print()
            print("Error_Pos_X: %.4f m" % (err_pos[0]))
            print("Error_Pos_Y: %.4f m" % (err_pos[1]))
            print("Error_Rot_Z: %.4f rad" % (err_rot))
            print()
            print("Lin_Vel_X: %.4f m/s" % (lin_vel_x))
            print("Lin_Vel_Y: %.4f m/s" % (lin_vel_y))
            print("Ang_Vel_Z: %.4f rad/s" % (ang_vel_z))
            print()
            # Generate and publish `cmd_vel` message
            self.cmd_vel_msg.linear.x = lin_vel_x
            self.cmd_vel_msg.linear.y = lin_vel_y
            self.cmd_vel_msg.angular.z = ang_vel_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            # Check if current waypoint is reached
            if numpy.linalg.norm(err_pos) < self.pos_tol and abs(err_rot) < self.rot_tol:
                # Upon reaching current waypoint, target the next waypoint
                self.wp_index += 1
                # If all waypoints have been visited, target the first waypoint again
                if self.wp_index >= self.wp_count:
                    self.wp_index = 0
            # Generate and publish `wp_index` message
            self.wp_index_msg.data = self.wp_index
            self.wp_index_pub.publish(self.wp_index_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.pid_g2g   = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        self.pid_sk_vx = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        self.pid_sk_vy = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        self.pid_sk_wz = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        self.goal_tol  = config['goal_tol'] # Goal tolerance dead-band of go-to-goal PID controller
        self.v_const   = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit   = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.pos_tol   = config['pos_tol'] # Position tolerance to determine whether goal is reached
        self.rot_tol   = config['rot_tol'] # Orientation tolerance to determine whether goal is reached
        self.config    = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_wayfinding')

    # Wayfinding class instance
    wayfinding_node = Wayfinding()

    # Dynamic reconfigure server
    wayfinding_node.dyn_reconf_srv = Server(WayfindingConfig, wayfinding_node.config_callback)

    # Messages
    wayfinding_node.wp_index_msg = Int32()
    wayfinding_node.cmd_vel_msg  = Twist()

    # Subscribers
    rospy.Subscriber('/vrx/wp_count', Int32, wayfinding_node.wp_count_callback)
    rospy.Subscriber('/wamv/cmd_pose', Pose, wayfinding_node.cmd_pose_callback)
    rospy.Subscriber('/wamv/cur_pose', Pose, wayfinding_node.cur_pose_callback)

    # Publishers
    wayfinding_node.wp_index_pub = rospy.Publisher('/vrx/wp_index', Int32, queue_size = 10)
    wayfinding_node.cmd_vel_pub  = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            wayfinding_node.wayfinding()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
