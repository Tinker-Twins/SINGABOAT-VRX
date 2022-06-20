#!/usr/bin/env python3

import math
import numpy
import rospy
import tf
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import WayfindingConfig
from singaboat_vrx.control_module import PIDController
from singaboat_vrx.common_utilities import gps_to_enu, quaternion_to_euler, euler_to_quaternion, normalize_angle

################################################################################

class Wayfinding:
    def __init__(self):
        # Initialize wayfinding
        self.cur_pos      = None # Current 2D position (x, y)
        self.cur_rot      = None # Current 1D orientation (yaw)
        self.cur_position = None # Current 3D position (x, y, z)
        self.cur_rotation = None # Current 3D orientation (roll, pitch, yaw)
        self.wps_pos_x    = [] # List of desired position (x) of all waypoints
        self.wps_pos_y    = [] # List of desired position (y) of all waypoints
        self.wps_rot_z    = [] # List of desired orientation (yaw) of all waypoints
        self.wps_position = [] # List of desired 3D position (x, y, z) of all waypoints
        self.wps_rotation = [] # List of desired 3D orientation (roll, pitch, yaw) of all waypoints
        self.wp_count     = None # Waypoint count (starts from 1)
        self.wp_index     = 0 # Waypoint index (starts from 0)
        self.cmd_pos      = None # Commanded 2D position (x, y)
        self.cmd_rot      = None # Commanded 1D orientation (yaw)
        self.time         = None # Current timestamp
        self.config       = {} # Wayfinding configuration
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

    def waypoint_callback(self, msg):
        for i in range(len(msg.poses)):
            if msg.poses: # Sanity check
                self.wp_count = len(msg.poses)
                waypoint = msg.poses[i] # Indexing starts from 0
                lat = waypoint.pose.position.latitude
                lon = waypoint.pose.position.longitude
                pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
                rot_x = quaternion_to_euler(waypoint.pose.orientation)[0]
                rot_y = quaternion_to_euler(waypoint.pose.orientation)[1]
                rot_z = quaternion_to_euler(waypoint.pose.orientation)[2]
                self.wps_pos_x.append(pos_x)
                self.wps_pos_y.append(pos_y)
                self.wps_rot_z.append(rot_z)
                self.wps_position.append(numpy.array([pos_x, pos_y, pos_z]))
                self.wps_rotation.append(euler_to_quaternion(rot_x, rot_y, rot_z))

    def wayfinding(self):
        # Broadcast transform of `wamv/base_link` frame w.r.t. `world` frame
        self.tf_broadcaster.sendTransform((self.cur_position[0], self.cur_position[1], self.cur_position[2]), (self.cur_rotation[0], self.cur_rotation[1], self.cur_rotation[2], self.cur_rotation[3]), rospy.Time.now(), 'wamv/base_link', 'world')
        # Broadcast transform of `waypoint_i` frame w.r.t. `world` frame
        for i in range(self.wp_count):
            frame_id = 'waypoint_' + str(i)
            self.tf_broadcaster.sendTransform((self.wps_position[i][0], self.wps_position[i][1], self.wps_position[i][2]), (self.wps_rotation[i][0], self.wps_rotation[i][1], self.wps_rotation[i][2], self.wps_rotation[i][3]), rospy.Time.now(), frame_id, 'world')
        # Proceed to current waypoint until it is actually reached
        self.cmd_pos = numpy.array([self.wps_pos_x[self.wp_index], self.wps_pos_y[self.wp_index]])
        self.cmd_rot = self.wps_rot_z[self.wp_index]
        self.time = rospy.get_time()
        if bool(self.config) and not [x for x in (self.cur_pos, self.cur_rot, self.cmd_pos, self.cmd_rot, self.wp_count, self.time) if x is None]:
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
                err_rot = normalize_angle(self.cmd_rot - self.cur_rot[2]) # Error in orientation
                ang_vel_z = self.pid_sk_wz.control(err_rot, self.time) # PID controller for Wz
            print("Wayfinding to waypoint {} of {}...".format(self.wp_index+1, self.wp_count)) # Counting and indexing starts from 1
            print("Current Target Waypoint: {:.4} m, {:.4} m, {:.4} rad".format(self.cmd_pos[0], self.cmd_pos[1], self.cmd_rot))
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
            # Check if current waypoint is reached
            if numpy.linalg.norm(err_pos) < self.pos_tol and abs(err_rot) < self.rot_tol:
                # Upon reaching current waypoint, target the next waypoint
                self.wp_index += 1
                # If all waypoints have been visited, target the first waypoint again
                if self.wp_index >= self.wp_count:
                    self.wp_index = 0

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
        self.pos_tol    = config['pos_tol'] # Position tolerance to determine whether goal is reached
        self.rot_tol    = config['rot_tol'] # Orientation tolerance to determine whether goal is reached
        self.debug      = config['debug'] # Flag to enable/disable debug messages
        self.config     = config
        return config

################################################################################

if __name__ == '__main__':
    rospy.init_node('singaboat_wayfinding')

    # Wayfinding class instance
    wayfinding_node = Wayfinding()

    # Dynamic reconfigure server
    wayfinding_node.dyn_reconf_srv = Server(WayfindingConfig, wayfinding_node.config_callback)

    # Transform broadcaster
    wayfinding_node.tf_broadcaster = tf.TransformBroadcaster()

    # Message
    wayfinding_node.cmd_vel_msg  = Twist()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, wayfinding_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, wayfinding_node.imu_callback)
    rospy.Subscriber('/vrx/wayfinding/waypoints', GeoPath, wayfinding_node.waypoint_callback)

    # Publisher
    wayfinding_node.cmd_vel_pub  = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size = 10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/vrx/wayfinding/waypoints', GeoPath)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            wayfinding_node.wayfinding()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
