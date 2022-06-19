#!/usr/bin/env python3

import numpy
import math
import copy
from scipy.misc import derivative
from dynamic_reconfigure.server import Server
import rospy
import actionlib
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Vector3Stamped, Twist
from sensor_msgs.msg import NavSatFix, Imu
from singaboat_vrx.cfg import MissionManagerConfig
from singaboat_vrx.msg import MissionResult, MissionFeedback, MissionAction
from singaboat_vrx.planning_module import Mission, DubinsPath, PathPlanner
from singaboat_vrx.control_module import PIDController
from singaboat_vrx.common_utilities import enu_to_gps, gps_to_enu, normalize_angle, quaternion_to_euler

################################################################################

class MissionManager:
    '''
    `MissionManager` is the highest abstraction layer of the navigation algorithm. It receives
    a set of waypoints and defines a path connecting them. It makes use of `Mission` and `PathPlanner`
    to perform the path following task of whole guidance strategy.
    '''
    def __init__(self):
        # Initialize mission manager
        self.action_server    = None # Action server
        self.feedback         = MissionFeedback() # Feedback from action server
        self.result           = MissionResult() # Result of action server
        self.mission          = None # Mission
        self.path_planner     = None # Path planner
        self.current_wp       = Mission.Waypoint(gps_lat = 0, gps_lon = 0) # Current waypoint
        self.target_wp        = Mission.Waypoint(gps_lat = 0, gps_lon = 0) # Target waypoint
        self.mission_complete = False # Boolean flag to check whether the mission has been completed
        self.angle_idx        = 0 # Angle index for scanning mode
        self.repulsion        = 0.0 # Repulsive field due to obstacles
        self.g2g_heading      = 0.0 # Go-to-goal heading
        self.cte              = 0.0 # Cross-track error
        self.he               = 0.0 # Heading error
        self.current_speed    = 0.0 # Current longitudinal linear velocity
        self.config           = {} # Mission manager configuration
        # ROS infrastructure
        self.cmd_vel_msg    = None
        self.cmd_vel_pub    = None
        self.dyn_reconf_srv = None

    def gps_callback(self, msg):
        if self.current_wp.pose.heading is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        x, y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        x += self.gps_offset * math.cos(self.current_wp.pose.heading)
        y += self.gps_offset * math.sin(self.current_wp.pose.heading)
        # Convert back to GPS coordinates
        self.current_wp.pose.gps_lat, self.current_wp.pose.gps_lon, _ = enu_to_gps(x, y)

    def speed_callback(self, msg):
        self.current_speed = numpy.linalg.norm([msg.vector.x, msg.vector.y])

    def imu_callback(self, msg):
        self.current_wp.pose.heading = quaternion_to_euler(msg.orientation)[2]

    def collision_avoidance_callback(self, msg):
        self.repulsion = msg.data

    def go_to_goal(self):
        '''
        Simple go-to-goal behavior using a PD Controller for angular velocity
        and an adaptive controller for longitudinal linear velocity.

        :param: None

        :return: None
        '''
        # Include the repulsion from local planner for online collision avoidance
        if self.repulsion != 0:
            self.g2g_heading = self.current_wp.pose.heading + self.repulsion
            self.max_speed = 1.67
            self.min_speed = 0.75
        else:
            self.g2g_heading = self.g2g_heading
            self.max_speed = 2.23
            self.min_speed = 1.0
        head_err = normalize_angle(self.g2g_heading - self.current_wp.pose.heading)
        # Head-to-goal PD Controller for angular velocity
        derivative = head_err - self.he
        ang_vel = self.h2g_kp*head_err + self.h2g_kd*derivative
        self.he = head_err
        # Adaptive controller for longitudinal linear velocity
        up = self.max_speed
        low = self.min_speed
        x_up = 0.15
        x_low = 0.4
        m = (up-low)/(x_up-x_low)
        b = up - m*x_up
        x = numpy.abs(head_err)
        lin_vel = numpy.clip(m*x+b,low,up)
        # Generate and publish `cmd_vel` message
        self.cmd_vel_msg.linear.x = lin_vel
        self.cmd_vel_msg.linear.y = 0.0
        self.cmd_vel_msg.angular.z = ang_vel
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def station_keeping(self, goal_wp):
        '''
        Station-keeping behavior using a cascaded PID controller for accurate control
        of linear and angular velocity.

        :param goal_wp: Goal waypoint pose

        :return e_pose: Error in pose
        '''
        # Convert goal waypoint coordinates to ENU format
        goal_wp.convert_to_enu()
        # Compute current and goal poses
        cur_pos = numpy.array([self.current_wp.pose.enu_x, self.current_wp.pose.enu_y])
        cur_rot = self.current_wp.pose.heading
        cmd_pos = numpy.array([goal_wp.pose.enu_x, goal_wp.pose.enu_y])
        cmd_rot = goal_wp.pose.heading
        # Compute error in pose based on VRX metric [e_pose = d + k^d * h]
        e_pose = numpy.linalg.norm(cmd_pos - cur_pos) + ((0.75 ** numpy.linalg.norm(cmd_pos - cur_pos)) * numpy.abs(normalize_angle(cmd_rot - cur_rot)))
        # Station-keeping control algorithm
        self.time = rospy.get_time()
        err_pos = cmd_pos - cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
        if numpy.linalg.norm(err_pos) > self.goal_tol: # (Euclidean distance to goal as L2 norm)
            # If far from goal, head directly towards the goal by controlling Vx & Wz
            lin_vel_x = numpy.linalg.norm(err_pos) * self.v_const # P controller for Vx
            if lin_vel_x > self.v_limit: # Clamp linear velocity along X-axis
                lin_vel_x = self.v_limit
            lin_vel_y = 0.0
            err_rot = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - cur_rot) # Error in orientation
            ang_vel_z = self.pid_g2g.control(err_rot, self.time) # PID controller for Wz
        else:
            # If near goal, perform fine adjustments in Vx, Vy & Wz for station-keeping
            rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - cur_rot) # G2G rotation transformation
            err_pos = numpy.array([numpy.linalg.norm(err_pos) * math.cos(rot_tf), numpy.linalg.norm(err_pos) * math.sin(rot_tf)]) # Error in position (in local frame)
            lin_vel_x = self.pid_sk_vx.control(err_pos[0], self.time) # PID controller for Vx
            lin_vel_y = self.pid_sk_vy.control(err_pos[1], self.time) # PID controller for Vy
            err_rot = normalize_angle(cmd_rot - cur_rot) # Error in orientation
            ang_vel_z = self.pid_sk_wz.control(err_rot, self.time) # PID controller for Wz
        # Generate and publish `cmd_vel` message
        if not [x for x in (lin_vel_x, lin_vel_y, ang_vel_z) if x is None]: # Sanity check
            self.cmd_vel_msg.linear.x = lin_vel_x
            self.cmd_vel_msg.linear.y = lin_vel_y
            self.cmd_vel_msg.angular.z = ang_vel_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
        return e_pose

    def scan_around(self, goal_wp, theta, delay):
        '''
        Swing left-and-right to search for any detectable patterns within the environment.

        :param goal_wp: Goal waypoint pose
        :param theta  : Current heading of the ASV
        :param delay  : Time delay to carry out scanning operation

        :return tgt_wp  : Target waypoint
        :return complete: Boolean flag determining weather the scanning operation has been completed
        '''
        tgt_wp = Mission.Waypoint(gps_lat = 0, gps_lon = 0)
        range_1 = numpy.linspace(-theta,theta,10)
        range_2 = numpy.linspace(theta,0,10)
        angle_range = numpy.hstack((range_1,range_2))
        complete = False
        self.angle_idx = self.angle_idx + 1
        i = self.angle_idx//delay
        tgt_wp.pose.gps_lat = goal_wp.pose.gps_lat
        tgt_wp.pose.gps_lon = goal_wp.pose.gps_lon
        if i >= angle_range.shape[0]:
            self.angle_idx = 0
            complete = True
            tgt_wp.pose.heading = goal_wp.pose.heading
        else:
            tgt_wp.pose.heading = goal_wp.pose.heading + angle_range[i]
        return tgt_wp, complete

    def update_path_planner(self):
        '''
        Main loop intended to update the output of the path planning algorithm.

        :param: None

        :return: None
        '''
        self.current_wp.convert_to_enu()
        mission_complete, self.g2g_heading, self.cte, self.target_wp = self.path_planner.update(current_wp = self.current_wp, speed = self.current_speed)
        if ((mission_complete == False) and ((self.target_wp.wp_mode == "PASS") or (self.target_wp.wp_mode == "NONE"))):
            self.go_to_goal()
        else:
            if ((self.current_wp.pose.gps_lat != 0) and (self.target_wp.wp_mode == "HALT")):
                e_pose = self.station_keeping(self.target_wp)
                if e_pose < self.pose_tol:
                    self.mission_complete = True
            elif ((self.current_wp.pose.gps_lat != 0) and (self.target_wp.wp_mode == "SCAN")):
                goal_wp, complete_pattern = self.scan_around(self.target_wp,math.pi/6, 30)
                self.station_keeping(goal_wp)
                if complete_pattern:
                    self.mission_complete = True
            elif ((self.current_wp.pose.gps_lat != 0) and (self.target_wp.wp_mode == "PARK")):
                e_pose = self.station_keeping(self.target_wp)
            else:
                self.mission_complete = True

    def load_mission(self, mission):
        '''
        Loads the external mission into a list of waypoints that can be undestood by the path planner.

        :param mission: The list of GeoPoseStamped waypoints provided by the high-level mission planner

        :return: None
        '''
        wps = []
        for p in mission:
            # Convert from quaternion to euler angle representation to extract the desired heading (yaw)
            quaternion = p.pose.orientation
            _, _, heading = quaternion_to_euler(quaternion)
            # Create the waypoint in the prescribed format
            wp = {'lat':p.pose.position.latitude,
                  'lon':p.pose.position.longitude,
                  'heading':heading,
                  'wp_mode':p.header.frame_id}
            # Create a list of dictionaries (making sure to avoid overwriting)
            wps.append(copy.deepcopy(wp))
        self.mission = Mission.Path(waypoints=wps)
        self.path_planner = PathPlanner(mission=self.mission, path_creator=DubinsPath)

    def mission_manager(self, goal):
        '''
        Main execution loop of the action server.

        :param goal: List of waypoints for the ASV to follow

        :return feedback, result: Feedback and the final action result
        '''
        self.load_mission(goal.mission)
        rate = rospy.Rate(10) # ROS rate
        while not self.mission_complete:
            # Check that preempt has not been requested by the client
            if self.action_server.is_preempt_requested():
                if self.debug:
                    print("{}: Preempted!".format('singaboat_mission_manager'))
                    print()
                self.action_server.set_preempted()
                self.result.mission_complete = False
                break
            # Call the main function of the path planner
            self.update_path_planner()
            self.feedback.mission_progress = 0.0
            # Publish the feedback
            self.action_server.publish_feedback(self.feedback)
            rate.sleep()
        # Set the correct responses whenever the mission is completed
        if self.mission_complete:
            self.mission_complete = False
            self.result.mission_complete = True
            if self.debug:
                print("{}: Succeeded!".format('singaboat_mission_manager'))
                print()
            self.action_server.set_succeeded(self.result)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.min_speed  = config['min_speed'] # Minimum longitudinal velocity of ASV
        self.max_speed  = config['max_speed'] # Maximum longitudinal velocity of ASV
        self.gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.pose_tol   = config['pose_tol'] # Pose tolerance to determine whether goal is reached
        self.h2g_kp     = config['H2G_kP'] # Proportional gain of head-to-goal PD controller
        self.h2g_kd     = config['H2G_kD'] # Derivative gain of head-to-goal PD controller
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

if __name__ == "__main__":
    rospy.init_node('singaboat_mission_manager', anonymous = True)

    # MissionManager class instance
    mission_manager_node = MissionManager()

    # Dynamic reconfigure server
    mission_manager_node.dyn_reconf_srv = Server(MissionManagerConfig, mission_manager_node.config_callback)

    # Message
    mission_manager_node.cmd_vel_msg = Twist()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, mission_manager_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped, mission_manager_node.speed_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, mission_manager_node.imu_callback)
    rospy.Subscriber('/wamv/obstacle_potential_field', Float32, mission_manager_node.collision_avoidance_callback)

    # Publisher
    mission_manager_node.cmd_vel_pub = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size=1)

    # Action server
    mission_manager_node.action_server = actionlib.SimpleActionServer('singaboat_mission_manager', MissionAction, execute_cb = mission_manager_node.mission_manager, auto_start = False)
    mission_manager_node.action_server.start()

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
