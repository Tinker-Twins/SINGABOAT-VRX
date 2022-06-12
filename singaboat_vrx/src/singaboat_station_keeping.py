#!/usr/bin/env python3

import math
import numpy
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, Twist # !!! NOT USED AS PER THE STANDARDS !!!
from singaboat_vrx.cfg import StationKeepingConfig
from singaboat_vrx.control_module import PIDController
from singaboat_vrx.common_utilities import normalize_angle

################################################################################

class StationKeeping:
    def __init__(self):
        # Initialize station-keeping
        self.cmd_pos = None # Desired 2D position (x, y)
        self.cmd_rot = None # Desired 1D orientation (yaw)
        self.cur_pos = None # Current 2D position (x, y)
        self.cur_rot = None # Current 1D orientation (yaw)
        self.time    = None # Current timestamp
        self.config  = {} # Station-keeping configuration
        # ROS infrastructure
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

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

    def station_keeping(self):
        # Control algorithm
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.cmd_pos, self.cmd_rot, self.cur_pos, self.cur_rot, self.time) if x is None]:
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

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.pid_g2g   = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        self.pid_sk_vx = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        self.pid_sk_vy = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        self.pid_sk_wz = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        self.goal_tol  = config['goal_tol'] # Goal tolerance dead-band of go-to-goal PID controller
        self.v_const   = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit   = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.config    = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_station_keeping')

    # StationKeeping class instance
    station_keeping_node = StationKeeping()

    # Dynamic reconfigure server
    station_keeping_node.dyn_reconf_srv = Server(StationKeepingConfig, station_keeping_node.config_callback)

    # Message
    station_keeping_node.cmd_vel_msg = Twist()

    # Subscribers
    rospy.Subscriber('/wamv/cmd_pose', Pose, station_keeping_node.cmd_pose_callback)
    rospy.Subscriber('/wamv/cur_pose', Pose, station_keeping_node.cur_pose_callback)

    # Publisher
    station_keeping_node.cmd_vel_pub = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            station_keeping_node.station_keeping()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
