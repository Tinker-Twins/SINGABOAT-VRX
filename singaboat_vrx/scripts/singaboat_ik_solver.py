#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import IKSolverConfig

################################################################################

class IKSolver():
    def __init__(self):
        # Initialize IK solver
        self.front_left_thruster_msg  = None
        self.front_right_thruster_msg = None
        self.rear_left_thruster_msg   = None
        self.rear_right_thruster_msg  = None
        self.front_left_thruster_pub  = None
        self.front_right_thruster_pub = None
        self.rear_left_thruster_pub   = None
        self.rear_right_thruster_pub  = None
        self.dyn_reconf_srv           = None
        self.config                   = {}

    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input
        return input

    def ik_solver_callback(self, msg):
        # Log IK solver input
        print("IK Solver I/P:")
        print("  Lin_Vel: %.4f m/s" % (msg.linear.x))
        print("  Ang_Vel: %.4f rad/s" % (msg.angular.z))
        # Comput inverse kinematics
        if msg.linear.x == 0.0 and msg.angular.z == 0.0: # Hydrodynamic brakes
            self.front_left_thruster_msg.data  = 1.0
            self.front_right_thruster_msg.data = 1.0
            self.rear_left_thruster_msg.data   = 1.0
            self.rear_right_thruster_msg.data  = 1.0
        else:
            self.rear_left_thruster_msg.data   = self.lin_vel_gain * msg.linear.x
            self.rear_right_thruster_msg.data  = self.lin_vel_gain * msg.linear.x
            self.rear_left_thruster_msg.data  += self.ang_vel_gain * (msg.angular.z * self.wamv_half_beam)
            self.rear_right_thruster_msg.data -= self.ang_vel_gain * (msg.angular.z * self.wamv_half_beam)
            self.front_left_thruster_msg.data  = -self.rear_left_thruster_msg.data
            self.front_right_thruster_msg.data = -self.rear_right_thruster_msg.data
        # Constrain the control commands
        self.front_left_thruster_msg.data  = self.constrain(self.front_left_thruster_msg.data, -1.0, 1.0)
        self.front_right_thruster_msg.data = self.constrain(self.front_right_thruster_msg.data, -1.0, 1.0)
        self.rear_left_thruster_msg.data   = self.constrain(self.rear_left_thruster_msg.data, -1.0, 1.0)
        self.rear_right_thruster_msg.data  = self.constrain(self.rear_right_thruster_msg.data, -1.0, 1.0)
        # Log IK solver output
        print("IK Solver O/P:")
        print("  FL_Thruster_Cmd: %.4f" % (self.front_left_thruster_msg.data))
        print("  FR_Thruster_Cmd: %.4f" % (self.front_right_thruster_msg.data))
        print("  RL_Thruster_Cmd: %.4f" % (self.rear_left_thruster_msg.data))
        print("  RR_Thruster_Cmd: %.4f" % (self.rear_right_thruster_msg.data))
        # Publish messages to WAM-V thrusters
        self.front_left_thruster_pub.publish(self.front_left_thruster_msg)
        self.front_right_thruster_pub.publish(self.front_right_thruster_msg)
        self.rear_left_thruster_pub.publish(self.rear_left_thruster_msg)
        self.rear_right_thruster_pub.publish(self.rear_right_thruster_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.lin_vel_gain   = config['lin_vel_gain']
        self.ang_vel_gain   = config['ang_vel_gain']
        self.wamv_half_beam = config['wamv_half_beam']
        self.config         = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_ik_solver', anonymous = True)

    # IKSolver class instance
    ik_solver_node = IKSolver()

    # Dynamic reconfigure server
    ik_solver_node.dyn_reconf_srv = Server(IKSolverConfig, ik_solver_node.config_callback)

    # Messages
    ik_solver_node.front_left_thruster_msg  = Float32()
    ik_solver_node.front_right_thruster_msg = Float32()
    ik_solver_node.rear_left_thruster_msg   = Float32()
    ik_solver_node.rear_right_thruster_msg  = Float32()

    # Subscriber
    rospy.Subscriber('/wamv/cmd_vel', Twist, ik_solver_node.ik_solver_callback)

    # Publishers
    ik_solver_node.front_left_thruster_pub  = rospy.Publisher('/wamv/thrusters/front_left_thruster_thrust_cmd', Float32, queue_size = 10)
    ik_solver_node.front_right_thruster_pub = rospy.Publisher('/wamv/thrusters/front_right_thruster_thrust_cmd', Float32, queue_size = 10)
    ik_solver_node.rear_left_thruster_pub   = rospy.Publisher('/wamv/thrusters/rear_left_thruster_thrust_cmd', Float32, queue_size = 10)
    ik_solver_node.rear_right_thruster_pub  = rospy.Publisher('/wamv/thrusters/rear_right_thruster_thrust_cmd', Float32, queue_size = 10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
