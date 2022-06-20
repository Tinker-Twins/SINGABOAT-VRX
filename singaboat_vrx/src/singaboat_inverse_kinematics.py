#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from singaboat_vrx.cfg import InverseKinematicsConfig
from singaboat_vrx.common_utilities import constrain

################################################################################

class InverseKinematics():
    def __init__(self):
        # Initialize inverse kinematics
        self.config = {} # Inverse kinematics configuration
        # ROS infrastructure
        self.center_thruster_msg = None
        self.left_thruster_msg   = None
        self.right_thruster_msg  = None
        self.center_thruster_pub = None
        self.left_thruster_pub   = None
        self.right_thruster_pub  = None
        self.dyn_reconf_srv      = None

    def inverse_kinematics_callback(self, msg):
        # Log IK input
        print("Lin Vel X: {:.4} m/s".format(msg.linear.x))
        print("Lin Vel Y: {:.4} m/s".format(msg.linear.y))
        print("Ang Vel Z: {:.4} rad/s".format(msg.angular.z))
        print()
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
        # Log IK output
        if self.debug:
            print("Thrust C: {:.4}".format(self.center_thruster_msg.data))
            print("Thrust L: {:.4}".format(self.left_thruster_msg.data))
            print("Thrust R: {:.4}".format(self.right_thruster_msg.data))
            print()
        # Publish messages to WAM-V thrusters
        self.center_thruster_pub.publish(self.center_thruster_msg)
        self.left_thruster_pub.publish(self.left_thruster_msg)
        self.right_thruster_pub.publish(self.right_thruster_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.thrust_factor = config['thrust_factor'] # Balancing factor for forward & reverse thrust forces
        self.drag_factor   = config['drag_factor'] # Balancing factor for longitudinal & lateral drag forces
        self.v_x_gain      = config['v_x_gain'] # Linear velocity gain along X-axis
        self.v_y_gain      = config['v_y_gain'] # Linear velocity gain along Y-axis
        self.w_z_gain      = config['w_z_gain'] # Angular velocity gain about Z-axis
        self.half_beam     = config['half_beam'] # Half beam width of WAM-V
        self.debug         = config['debug'] # Flag to enable/disable debug messages
        self.config        = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_inverse_kinematics', anonymous = True)

    # InverseKinematics class instance
    inverse_kinematics_node = InverseKinematics()

    # Dynamic reconfigure server
    inverse_kinematics_node.dyn_reconf_srv = Server(InverseKinematicsConfig, inverse_kinematics_node.config_callback)

    # Messages
    inverse_kinematics_node.center_thruster_msg = Float32()
    inverse_kinematics_node.left_thruster_msg   = Float32()
    inverse_kinematics_node.right_thruster_msg  = Float32()

    # Subscriber
    rospy.Subscriber('/wamv/cmd_vel', Twist, inverse_kinematics_node.inverse_kinematics_callback)

    # Publishers
    inverse_kinematics_node.center_thruster_pub = rospy.Publisher('/wamv/thrusters/center_thruster_thrust_cmd', Float32, queue_size = 10)
    inverse_kinematics_node.left_thruster_pub   = rospy.Publisher('/wamv/thrusters/left_thruster_thrust_cmd', Float32, queue_size = 10)
    inverse_kinematics_node.right_thruster_pub  = rospy.Publisher('/wamv/thrusters/right_thruster_thrust_cmd', Float32, queue_size = 10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
