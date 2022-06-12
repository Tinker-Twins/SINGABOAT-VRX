#!/usr/bin/env python3

import rospy
import math
import numpy
from singaboat_vrx.singaboat_pid_controller import PIDController
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from singaboat_vrx.msg import Pose3D
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import WayfindingConfig

################################################################################

class Wayfinding:
    def __init__(self):
        self.cmd_pos = None # Desired 2D position (x, y)
        self.cmd_rot = None # Desired 1D orientation (yaw)
        self.cur_pos = None # Current 2D position (x, y)
        self.cur_rot = None # Current 1D orientation (yaw)
        self.wp_num  = None # Number of waypoints
        self.wp_idx  = 0 # Waypoint index (starts from 0)
        self.time    = None # Current timestamp
        self.config  = {} # Controller configuration (kP, kI, kD, kS)
        # ROS infrastructure
        self.wp_idx_msg     = None
        self.cmd_vel_msg    = None
        self.wp_idx_pub     = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

    def wp_num_callback(self, msg):
        self.wp_num  = msg.data

    def cmd_pose_callback(self, msg):
        pos_x = msg.pos_x
        pos_y = msg.pos_y
        rot_z = msg.rot_z
        self.cmd_pos = numpy.array([pos_x, pos_y])
        self.cmd_rot = rot_z

    def cur_pose_callback(self, msg):
        pos_x = msg.pos_x
        pos_y = msg.pos_y
        rot_z = msg.rot_z
        self.cur_pos = numpy.array([pos_x, pos_y])
        self.cur_rot = rot_z

    def wayfinding(self):
        # Proceed to current waypoint until it is actually reached
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.wp_num, self.cmd_pos, self.cmd_rot, self.cur_pos, self.cur_rot, self.time) if x is None]:
            # Error in position [x_des - x_cur, y_des - y_cur]
            err_pos = self.cmd_pos - self.cur_pos
            # Error in orientation
            if numpy.linalg.norm(err_pos) > self.goal_tol: # (Euclidean distance to goal as L2 norm)
                # If far from goal, ignore desired (goal) orientation, head towards goal
                lin_vel = numpy.linalg.norm(err_pos) * self.v_const
                err_rot = math.atan2(err_pos[1], err_pos[0]) - self.cur_rot
            else:
                # Stop and orient according to desired orientation
                lin_vel = 0
                err_rot = self.cmd_rot - self.cur_rot
            err_rot = (err_rot + math.pi) % (2 * math.pi) - math.pi
            ang_vel = self.pid_g2g.control(err_rot, self.time)
            if lin_vel > self.v_limit: # Clamp linear velocity
                lin_vel = self.v_limit
            print("WP_Number: %d" % (self.wp_num))
            print("WP_Index: %d" % (self.wp_idx))
            print("Error_Pos: %.4f m" % (numpy.linalg.norm(err_pos)))
            print("Error_Rot: %.4f rad" % (err_rot))
            print("Lin_Vel: %.4f m/s" % (lin_vel))
            print("Ang_Vel: %.4f rad/s" % (ang_vel))
            # Generate and publish `cmd_vel` message
            self.cmd_vel_msg.linear.x  = lin_vel
            self.cmd_vel_msg.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            # Check if current waypoint is reached
            if numpy.linalg.norm(err_pos) < self.pos_tol and abs(err_rot) < self.rot_tol:
                # Upon reaching current waypoint, target the next waypoint
                self.wp_idx += 1
                # If all waypoints have been visited, target the first waypoint again
                if self.wp_idx >= self.wp_num:
                    self.wp_idx = 0
            # Generate and publish `wp_idx` message
            self.wp_idx_msg.data = self.wp_idx
            self.wp_idx_pub.publish(self.wp_idx_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.pid_g2g = PIDController(config['kP'], config['kI'], config['kD'], config['kS']) # Go-to-goal PID controller
        self.goal_tol = config['goal_tol'] # Goal tolerance dead-band of PID controller
        self.pos_tol = config['pos_tol'] # Position tolerance to determine whether goal is reached
        self.rot_tol = config['rot_tol'] # Orientation tolerance to determine whether goal is reached
        self.v_const = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit  = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.config  = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_wayfinding')

    # Wayfinding class instance
    wayfinding_node = Wayfinding()

    # Dynamic reconfigure server
    wayfinding_node.dyn_reconf_srv = Server(WayfindingConfig, wayfinding_node.config_callback)

    # Messages
    wayfinding_node.wp_idx_msg   = Int32()
    wayfinding_node.cmd_vel_msg  = Twist()

    # Subscribers
    rospy.Subscriber('/vrx/wp_num', Int32, wayfinding_node.wp_num_callback)
    rospy.Subscriber('/wamv/cmd_pose', Pose3D, wayfinding_node.cmd_pose_callback)
    rospy.Subscriber('/wamv/cur_pose', Pose3D, wayfinding_node.cur_pose_callback)

    # Publishers
    wayfinding_node.wp_idx_pub   = rospy.Publisher('/vrx/wp_idx', Int32, queue_size = 10)
    wayfinding_node.cmd_vel_pub  = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            wayfinding_node.wayfinding()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
