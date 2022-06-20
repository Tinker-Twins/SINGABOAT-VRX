#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from singaboat_vrx.cfg import IKSolverConfig
from singaboat_vrx.common_utilities import constrain

################################################################################

class IKSolver():
    def __init__(self):
        # Initialize IK solver
        self.config = {} # IK solver configuration
        # ROS infrastructure
        self.center_thruster_msg = None
        self.left_thruster_msg   = None
        self.right_thruster_msg  = None
        self.center_thruster_pub = None
        self.left_thruster_pub   = None
        self.right_thruster_pub  = None
        self.dyn_reconf_srv      = None

    def ik_solver_callback(self, msg):
        # Log IK solver input
        print("IK Solver I/P:")
        print("  Lin_Vel_X: %.4f m/s" % (msg.linear.x))
        print("  Lin_Vel_Y: %.4f m/s" % (msg.linear.y))
        print("  Ang_Vel_Z: %.4f rad/s" % (msg.angular.z))
        # Comput inverse kinematics
        # Lin_Vel_X
        if msg.linear.x >= 0.0:
            self.left_thruster_msg.data  = self.v_x_gain * msg.linear.x
            self.right_thruster_msg.data = self.v_x_gain * msg.linear.x
        else:
            self.left_thruster_msg.data  = self.thrust_factor * self.v_x_gain * msg.linear.x
            self.right_thruster_msg.data = self.thrust_factor * self.v_x_gain * msg.linear.x
        # Lin_Vel_Y
        if msg.linear.y >= 0.0:
            self.center_thruster_msg.data = self.drag_factor * self.v_y_gain * msg.linear.y
        else:
            self.center_thruster_msg.data = self.thrust_factor * self.drag_factor * self.v_y_gain * msg.linear.y
        # Ang_Vel_Z
        self.left_thruster_msg.data  -= self.w_z_gain * (self.half_beam * msg.angular.z)
        self.right_thruster_msg.data += self.w_z_gain * (self.half_beam * msg.angular.z)
        # Constrain the control commands
        self.center_thruster_msg.data = constrain(self.center_thruster_msg.data, -1.0, 1.0)
        self.left_thruster_msg.data   = constrain(self.left_thruster_msg.data, -1.0, 1.0)
        self.right_thruster_msg.data  = constrain(self.right_thruster_msg.data, -1.0, 1.0)
        # Log IK solver output
        print("IK Solver O/P:")
        print("  Thrust_C: %.4f" % (self.center_thruster_msg.data))
        print("  Thrust_L: %.4f" % (self.left_thruster_msg.data))
        print("  Thrust_R: %.4f" % (self.right_thruster_msg.data))
        # Publish messages to WAM-V thrusters
        self.center_thruster_pub.publish(self.center_thruster_msg)
        self.left_thruster_pub.publish(self.left_thruster_msg)
        self.right_thruster_pub.publish(self.right_thruster_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.thrust_factor = config['thrust_factor']
        self.drag_factor   = config['drag_factor']
        self.v_x_gain      = config['v_x_gain']
        self.v_y_gain      = config['v_y_gain']
        self.w_z_gain      = config['w_z_gain']
        self.half_beam     = config['half_beam']
        self.config        = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_ik_solver', anonymous = True)

    # IKSolver class instance
    ik_solver_node = IKSolver()

    # Dynamic reconfigure server
    ik_solver_node.dyn_reconf_srv = Server(IKSolverConfig, ik_solver_node.config_callback)

    # Messages
    ik_solver_node.center_thruster_msg = Float32()
    ik_solver_node.left_thruster_msg   = Float32()
    ik_solver_node.right_thruster_msg  = Float32()

    # Subscriber
    rospy.Subscriber('/wamv/cmd_vel', Twist, ik_solver_node.ik_solver_callback)

    # Publishers
    ik_solver_node.center_thruster_pub = rospy.Publisher('/wamv/thrusters/center_thruster_thrust_cmd', Float32, queue_size = 10)
    ik_solver_node.left_thruster_pub   = rospy.Publisher('/wamv/thrusters/left_thruster_thrust_cmd', Float32, queue_size = 10)
    ik_solver_node.right_thruster_pub  = rospy.Publisher('/wamv/thrusters/right_thruster_thrust_cmd', Float32, queue_size = 10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
