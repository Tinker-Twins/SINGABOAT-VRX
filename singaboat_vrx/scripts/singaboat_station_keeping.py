#!/usr/bin/env python3

import rospy
import math
import numpy
from singaboat_vrx.singaboat_pid_controller import PIDController
from geometry_msgs.msg import Twist
from singaboat_vrx.msg import Pose3D
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import StationKeepingConfig

################################################################################

class StationKeeping:
    def __init__(self):
        self.cmd_pos = None # Desired 2D position (x, y)
        self.cmd_rot = None # Desired 1D orientation (yaw)
        self.cur_pos = None # Current 2D position (x, y)
        self.cur_rot = None # Current 1D orientation (yaw)
        self.time    = None # Current timestamp
        self.config  = {} # Controller configuration (kP, kI, kD, kS)
        # ROS infrastructure
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

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

    def station_keeping(self):
        # Control algorithm
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.cmd_pos, self.cmd_rot, self.cur_pos, self.cur_rot, self.time) if x is None]:
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
            #print("Cur_Pos:" + str(self.cur_pos))
            #print("Cmd_Pos:" + str(self.cmd_pos))
            #print("Cur_Rot: %.4f rad" % (self.cur_rot))
            #print("G2G_Rot: %.4f rad" % (math.atan2(err_pos[1], err_pos[0])))
            #print("Cmd_Rot: %.4f rad" % (self.cmd_rot))
            print("Error_Pos: %.4f m" % (numpy.linalg.norm(err_pos)))
            print("Error_Rot: %.4f rad" % (err_rot))
            print("Lin_Vel: %.4f m/s" % (lin_vel))
            print("Ang_Vel: %.4f rad/s" % (ang_vel))
            # Generate and publish `cmd_vel` message
            self.cmd_vel_msg.linear.x  = lin_vel
            self.cmd_vel_msg.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.pid_g2g  = PIDController(config['kP'], config['kI'], config['kD'], config['kS']) # Go-to-goal PID controller
        self.goal_tol = config['goal_tol'] # Goal tolerance dead-band of PID controller
        self.v_const  = config['v_const'] # Proportional gain for linear velocity outside goal tolerance
        self.v_limit  = config['v_limit'] # Saturation limit for linear velocity outside goal tolerance
        self.config   = config
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
    rospy.Subscriber('/wamv/cmd_pose', Pose3D, station_keeping_node.cmd_pose_callback)
    rospy.Subscriber('/wamv/cur_pose', Pose3D, station_keeping_node.cur_pose_callback)

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
