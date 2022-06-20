#!/usr/bin/env python3

import numpy
import math
import time
import rospy
import actionlib
from dynamic_reconfigure.server import Server
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from vrx_gazebo.msg import Task
from usv_msgs.msg import RangeBearing
from singaboat_vrx.msg import MissionGoal, MissionAction
from singaboat_vrx.cfg import GymkhanaChallengeConfig
from singaboat_vrx.planning_module import FiniteStateMachine
from singaboat_vrx.common_utilities import gps_to_enu, enu_to_gps, quaternion_to_euler, euler_to_quaternion, heading_to_bearing

################################################################################

class GymkhanaChallenge:
    def __init__(self):
        # Initialize gymkhana challenge
        self.fsm            = FiniteStateMachine() # FiniteStateMachine class instance
        self.next_gate_pose = GeoPoseStamped() # Keeps track of pose (lat, lon, quat) of the next channel gate
        self.exit_detected  = False # Boolean flag to check whether channel exit has been detected
        self.asv_pos_lat    = None # ASV position latitude
        self.asv_pos_lon    = None # ASV position longitude
        self.asv_heading    = None # ASV orientation (yaw)
        self.pinger_range   = None # Range measurement from acoustic pinger locater (APL)
        self.pinger_bearing = None # Bearing measurement from acoustic pinger locater (APL)
        self.state_time     = rospy.get_time() # State initialization time
        self.start_time     = time.time() # Records time at the start of algorithm
        self.config         = {} # Gymkhana challenge configuration
        # ROS infrastructure
        self.potential_field_msg = None
        self.potential_field_pub = None
        self.dyn_reconf_srv      = None

    def gps_callback(self, msg):
        if self.asv_heading is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        # Convert to ENU coordinates for offset correction
        x, y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        x += self.gps_offset * math.cos(self.asv_heading)
        y += self.gps_offset * math.sin(self.asv_heading)
        # Convert back to GPS coordinates
        self.asv_pos_lat, self.asv_pos_lon, _ = enu_to_gps(x, y)

    def imu_callback(self, msg):
        self.asv_heading = quaternion_to_euler(msg.orientation)[2]

    def apl_callback(self, msg):
        self.pinger_range = msg.range
        self.pinger_bearing = msg.bearing
        goal_tol = self.max_goal_tol if self.fsm.current_state == 'Find Pinger' else self.min_goal_tol
        if ((self.fsm.current_state == 'Find Pinger') or
            (self.fsm.current_state == 'Hold Position')):
            elapsed = time.time() - self.start_time
            if elapsed > 15.0:
                if numpy.abs(msg.range) > goal_tol:
                    self.action_client.cancel_all_goals()
                    self.start_time = time.time()

    def channel_navigation_callback(self, msg):
        if ((self.fsm.current_state == 'Initialize') and (msg.header.frame_id == 'Entrance')):
            self.next_gate_pose = msg
            self.action_client.cancel_all_goals() # Cancel current mission
        elif ((self.fsm.current_state == 'Enter Channel') and (msg.header.frame_id == 'Gate 1')):
            self.next_gate_pose = msg
            self.action_client.cancel_all_goals() # Cancel current mission
        elif ('Pass Gate ' in self.fsm.current_state):
            if ('Gate ' in msg.header.frame_id):
                self.next_gate_pose = msg
                self.action_client.cancel_all_goals() # Cancel current mission
            elif (msg.header.frame_id == 'Exit'):
                self.next_gate_pose = msg
                self.exit_detected = True
                self.action_client.cancel_all_goals() # Cancel current mission
        else:
            pass

    def collision_avoidance_callback(self, msg):
        obstacles = numpy.array(msg.data) # Load obstacle data (position coordinates of the obstacles w.r.t. WAM-V)
        obstacles = obstacles.reshape(-1, 2) # Reshape to club position coordinates of the obstacles together
        # Variables used to compute repulsive field (according to potential field algorithm) due to obstacles
        w = numpy.array([]) # Stores reciprocal of distance to each obstacle w.r.t. WAM-V (weighing terms)
        y = numpy.array([]) # Stores lateral offset of each obstacle w.r.t. WAM-V (data points to be averaging)
        # Tolerance for collision avoidance (distance of obstacles w.r.t. WAM-V)
        collision_tol = self.collision_tol if self.fsm.current_state == 'Find Pinger' else 0.0
        for obs_x, obs_y in obstacles:
            obs_dist  = numpy.linalg.norm([obs_x, obs_y]) # Euclidean distance between WAM-V and obstacle (as L2 norm)
            obs_dirxn = numpy.arctan2(obs_y, obs_x) # Direction of obstacle w.r.t. WAM-V
            # If the obstacle is within a certain range and field-of-view, WAM-V may collide with it
            if ((obs_dist < collision_tol) and (numpy.abs(obs_dirxn) < numpy.deg2rad(self.collision_fov))):
                w = numpy.append(w, 1/obs_dist)
                y = numpy.append(y, obs_y)
        if w.shape[0] > 0: # Only if there is any obstacle that the WAM-V may collide with
            wt_avg = numpy.sum(w*y)/numpy.sum(w) # Compute weighted average
            repulsion = numpy.clip(-4.0/wt_avg, -1.0, 1.0) # Compute repulsive field (according to potential field algorithm) due to obstacles
            if self.debug:
                print('Obstacles detected towards {}, correcting course by {:.4f} rad'.format('left' if wt_avg > 0 else 'right', self.potential_field_msg.data))
                print()
        else:
            repulsion = 0.0 # Reset repulsive field
        self.potential_field_msg.data = repulsion
        self.potential_field_pub.publish(self.potential_field_msg)

    def create_path(self, end_wp, start_wp_mode='PASS', end_wp_mode='SCAN'):
        '''
        Create a path from the current position of ASV to the given end waypoint.

        :param end_wp       : Waypoint defining the final (end) pose
        :param start_wp_mode: Mode of the first waypoint of the path
        :param end_wp_mode  : Mode of the last waypoint of the path

        :return path: Path comprising of a series of waypoints
        '''
        # Start waypoint
        start_wp = GeoPoseStamped()
        start_wp.header.frame_id = start_wp_mode
        start_wp.pose.position.latitude = self.asv_pos_lat
        start_wp.pose.position.longitude = self.asv_pos_lon
        asv_quat = euler_to_quaternion(0, 0, self.asv_heading)
        start_wp.pose.orientation.x = asv_quat[0]
        start_wp.pose.orientation.y = asv_quat[1]
        start_wp.pose.orientation.z = asv_quat[2]
        start_wp.pose.orientation.w = asv_quat[3]
        # End waypoint
        end_wp.header.frame_id = end_wp_mode
        # Create a simple two-waypoint path (straight-line path)
        path = [start_wp, end_wp]
        return path

    def navigate(self, path):
        '''
        Send path from this node (action client) to the mission manager (action server).

        :param path: Path comprising of a series of waypoints

        :return: None
        '''
        goal = MissionGoal(mission=path) # Generate action goal (i.e. mission goal)
        self.action_client.send_goal(goal) # Send action goal (i.e. mission goal) to the action server (i.e. mission manager)
        self.action_client.wait_for_result() # Wait for result from the action server (i.e. mission manager)

    def locate_pinger(self, asv_lat, asv_lon, range, bearing, earth_rad=6371008):
        '''
        Compute location (lat, lon) of the acoustic pinger contained within a black box
        based on the position (lat, lon) of ASV and measurements (range and bearing)
        from the acoustic pinger locator (APL).

        :param asv_lat  : Latitude of the ASV in degrees
        :param asv_lon  : Longitude of the ASV in degrees
        :param range    : Range measurement from the APL (distance to travel)
        :param bearing  : Bearing measurement from the APL (direction of travel)
        :param earth_rad: Radius of Earth

        :return pinger_lat, pinger_lon: Position (lat, lon) of the black box
        '''
        # Convert source point latitude and longitude, and desired bearing to radians
        asv_lat = numpy.radians(asv_lat)
        asv_lon = numpy.radians(asv_lon)
        bearing = numpy.radians(bearing)
        range = range/earth_rad
        # Calculate destination point latitude
        pinger_lat = numpy.rad2deg(numpy.arcsin(numpy.sin(asv_lat)*numpy.cos(range) + numpy.cos(asv_lat)*numpy.sin(range)*numpy.cos(bearing)))
        # Calculate destination point longitude
        pinger_lon = numpy.rad2deg(asv_lon + numpy.arctan2(numpy.sin(bearing)*numpy.sin(range)*numpy.cos(asv_lat), numpy.cos(range)-numpy.sin(asv_lat)*numpy.sin(pinger_lat)))
        return pinger_lat, pinger_lon

    def initial_state_handler(self):
        '''
        State handler for initialization. The ASV is commanded to stay where it is and search the entry gate.
        This state prevails until the channel entrance is detected.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Construct the goal waypoint (current pose of the ASV)
        goal = GeoPoseStamped()
        goal.pose.position.latitude = self.asv_pos_lat
        goal.pose.position.longitude = self.asv_pos_lon
        asv_quat = euler_to_quaternion(0, 0, self.asv_heading)
        goal.pose.orientation.x = asv_quat[0]
        goal.pose.orientation.y = asv_quat[1]
        goal.pose.orientation.z = asv_quat[2]
        goal.pose.orientation.w = asv_quat[3]
        # Stay at the current location and search for channel entrance
        path = self.create_path(goal, start_wp_mode='PASS', end_wp_mode='SCAN')
        self.navigate(path)
        # Define the next state
        if rospy.get_time() - self.state_time >= 60: # 60 seconds to detect entry gate, else find pinger
            if self.debug:
                print("Stuck in the channel, aborting channel navigation...")
                print()
            next_state = 'Find Pinger'
        else:
            self.state_time = rospy.get_time()
            next_state = 'Enter Channel'
        return next_state

    def entrance_state_handler(self):
        '''
        State handler for entering the channel. The ASV has to navigate towards the center of entry gate.
        This state prevails until the next channel gate or channel exit is detected.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Navigate towards the center of entry gate and search for gate 1 or channel exit
        path = self.create_path(self.next_gate_pose, start_wp_mode='PASS', end_wp_mode='SCAN')
        self.navigate(path)
        # Define the next state
        if rospy.get_time() - self.state_time >= 60: # 60 seconds to detect gate 1, else find pinger
            if self.debug:
                print("Stuck in the channel, aborting channel navigation...")
                print()
            next_state = 'Find Pinger'
        else:
            self.state_time = rospy.get_time()
            next_state = 'Exit Channel' if self.exit_detected else 'Pass Gate 1'
        return next_state

    def gate_1_state_handler(self):
        '''
        State handler for passing through the channel gate 1. The ASV has to navigate towards the center of gate 1.
        This state prevails until the next channel gate or channel exit is detected.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Navigate towards the center of gate 1 and search for gate 2 or channel exit
        path = self.create_path(self.next_gate_pose, start_wp_mode='PASS', end_wp_mode='SCAN')
        self.navigate(path)
        # Define the next state
        if rospy.get_time() - self.state_time >= 60: # 60 seconds to detect gate 2, else find pinger
            if self.debug:
                print("Stuck in the channel, aborting channel navigation...")
                print()
            next_state = 'Find Pinger'
        else:
            self.state_time = rospy.get_time()
            next_state = 'Exit Channel' if self.exit_detected else 'Pass Gate 2'
        return next_state

    def gate_2_state_handler(self):
        '''
        State handler for passing through the channel gate 2. The ASV has to navigate towards the center of gate 2.
        This state prevails until the next channel gate or channel exit is detected.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Navigate towards the center of gate 2 and search for gate 3 or channel exit
        path = self.create_path(self.next_gate_pose, start_wp_mode='PASS', end_wp_mode='SCAN')
        self.navigate(path)
        # Define the next state
        if rospy.get_time() - self.state_time >= 60: # 60 seconds to detect gate 3, else find pinger
            if self.debug:
                print("Stuck in the channel, aborting channel navigation...")
                print()
            next_state = 'Find Pinger'
        else:
            self.state_time = rospy.get_time()
            next_state = 'Exit Channel' if self.exit_detected else 'Pass Gate 3'
        return next_state

    def gate_3_state_handler(self):
        '''
        State handler for passing through the channel gate 3. The ASV has to navigate towards the center of gate 3.
        This state prevails until the next channel gate or channel exit is detected.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Navigate towards the center of gate 3 and search for channel exit
        path = self.create_path(self.next_gate_pose, start_wp_mode='PASS', end_wp_mode='SCAN')
        self.navigate(path)
        # Define the next state
        if rospy.get_time() - self.state_time >= 60: # 60 seconds to detect exit gate, else find pinger
            if self.debug:
                print("Stuck in the channel, aborting channel navigation...")
                print()
            next_state = 'Find Pinger'
        else:
            self.state_time = rospy.get_time()
            next_state = 'Exit Channel' if self.exit_detected else 'Pass Gate 3'
        return next_state

    def exit_state_handler(self):
        '''
        State handler for exiting the channel. The ASV has to navigate towards the center of exit gate.
        This state prevails until the ASV has exited the channel.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Navigate towards the center of exit gate
        path = self.create_path(self.next_gate_pose, start_wp_mode='PASS', end_wp_mode='PASS')
        self.navigate(path)
        # Define the next state
        next_state = 'Find Pinger'
        return next_state

    def find_pinger_state_handler(self):
        '''
        State handler for finding the pinger. The ASV has to navigate towards the pinger while avoiding the obstacle buoys.
        This state prevails until the ASV has reached in close vicinity of the pinger.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Construct the goal waypoint (pinger location)
        goal = GeoPoseStamped()
        goal_quat = euler_to_quaternion(0, 0, 0) # Goal orientation
        pinger_range = self.pinger_range # Pinger range w.r.t. APL frame
        asv_bearing = heading_to_bearing(self.asv_heading) # Convert ASV heading to compass bearing
        pinger_bearing = asv_bearing - numpy.rad2deg(self.pinger_bearing) # Compute pinger bearing w.r.t. global frame
        pinger_lat, pinger_lon = self.locate_pinger(self.asv_pos_lat, self.asv_pos_lon, pinger_range, pinger_bearing) # Compute GPS coordinates (lat, lon) of the pinger
        # Print pinger location
        if self.debug:
            print("Object Name: Pinger")
            print("Object Location: {}° N {}° E".format(pinger_lat, pinger_lon))
            print()
        goal.pose.position.latitude = pinger_lat
        goal.pose.position.longitude = pinger_lon
        goal.pose.orientation.x = goal_quat[0]
        goal.pose.orientation.y = goal_quat[1]
        goal.pose.orientation.z = goal_quat[2]
        goal.pose.orientation.w = goal_quat[3]
        if pinger_range > 0:
            # Navigate towards the pinger
            path = self.create_path(goal, start_wp_mode='PASS', end_wp_mode='HALT')
            self.navigate(path)
            # Define the next state
            next_state = 'Hold Position' if numpy.abs(self.pinger_range) < self.max_goal_tol else 'Find Pinger'
        return next_state

    def station_keeping_state_handler(self):
        '''
        State handler for station-keeping over the pinger. The ASV has to maintian its position over the pinger.
        This state prevails until the end of the task.

        :param: None

        :return next_state: Name of the next state (depending on the transition logic)
        '''
        # Construct the goal waypoint (pinger in black box)
        goal = GeoPoseStamped()
        goal_quat = euler_to_quaternion(0, 0, 0) # Goal orientation
        pinger_range = self.min_goal_tol # Pinger range w.r.t. APL frame [currently set constant since actual readings are very noisy and hence the controller generates erratic commands]
        asv_bearing = heading_to_bearing(self.asv_heading) # Convert ASV heading to compass bearing
        pinger_bearing = asv_bearing - numpy.rad2deg(self.pinger_bearing) # Compute pinger bearing w.r.t. global frame
        pinger_lat, pinger_lon = self.locate_pinger(self.asv_pos_lat, self.asv_pos_lon, pinger_range, pinger_bearing) # Compute GPS coordinates (lat, lon) of the pinger
        goal.pose.position.latitude = pinger_lat
        goal.pose.position.longitude = pinger_lon
        goal.pose.orientation.x = goal_quat[0]
        goal.pose.orientation.y = goal_quat[1]
        goal.pose.orientation.z = goal_quat[2]
        goal.pose.orientation.w = goal_quat[3]
        # Station-keeping over the pinger
        path = self.create_path(goal, start_wp_mode='PASS', end_wp_mode='PARK')
        self.navigate(path)
        # Define the next state
        next_state = 'Hold Position'
        return next_state

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.gps_offset    = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.min_goal_tol  = config['min_goal_tol'] # Minimum goal tolerance (for station-keeping)
        self.max_goal_tol  = config['max_goal_tol'] # Maximum goal tolerance (for finding pinger)
        self.collision_tol = config['collision_tol'] # Obstacle range tolerance for collision avoidance
        self.collision_fov = config['collision_fov'] # Obstacle field of view for collision avoidance
        self.debug         = config['debug'] # Flag to enable/disable debug messages
        self.config        = config
        return config

################################################################################

if __name__ == "__main__":
    rospy.init_node('singaboat_gymkhana_challenge', anonymous = True)

    # GymkhanaChallenge class instance
    gymkhana_challenge_node = GymkhanaChallenge()

    # Dynamic reconfigure server
    gymkhana_challenge_node.dyn_reconf_srv = Server(GymkhanaChallengeConfig, gymkhana_challenge_node.config_callback)

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, gymkhana_challenge_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, gymkhana_challenge_node.imu_callback)
    rospy.Subscriber('/wamv/sensors/pingers/pinger/range_bearing', RangeBearing, gymkhana_challenge_node.apl_callback)
    rospy.Subscriber('/wamv/detected_objects', GeoPoseStamped, gymkhana_challenge_node.channel_navigation_callback)
    rospy.Subscriber('/wamv/detected_obstacles', Float64MultiArray, gymkhana_challenge_node.collision_avoidance_callback)

    # Message
    gymkhana_challenge_node.potential_field_msg = Float32()

    # Publisher
    gymkhana_challenge_node.potential_field_pub = rospy.Publisher('/wamv/obstacle_potential_field', Float32, queue_size=1)

    # Action client
    gymkhana_challenge_node.action_client = actionlib.SimpleActionClient('singaboat_mission_manager', MissionAction)
    gymkhana_challenge_node.action_client.wait_for_server()

    try:
        # Initialize finite state machine
        init_hdlr = gymkhana_challenge_node.initial_state_handler
        entry_hdlr = gymkhana_challenge_node.entrance_state_handler
        gate1_hdlr = gymkhana_challenge_node.gate_1_state_handler
        gate2_hdlr = gymkhana_challenge_node.gate_2_state_handler
        gate3_hdlr = gymkhana_challenge_node.gate_3_state_handler
        exit_hdlr = gymkhana_challenge_node.exit_state_handler
        pinger_hdlr = gymkhana_challenge_node.find_pinger_state_handler
        park_hdlr = gymkhana_challenge_node.station_keeping_state_handler
        gymkhana_challenge_node.fsm.add_state(state_name='Initialize',    state_hdlr=init_hdlr,   start_state=True, debug_info='Initializing...')
        gymkhana_challenge_node.fsm.add_state(state_name='Enter Channel', state_hdlr=entry_hdlr,  exit_hdlr=None,   debug_info='Entering channel...')
        gymkhana_challenge_node.fsm.add_state(state_name='Pass Gate 1',   state_hdlr=gate1_hdlr,  exit_hdlr=None,   debug_info='Passing through gate 1...')
        gymkhana_challenge_node.fsm.add_state(state_name='Pass Gate 2',   state_hdlr=gate2_hdlr,  exit_hdlr=None,   debug_info='Passing through gate 2...')
        gymkhana_challenge_node.fsm.add_state(state_name='Pass Gate 3',   state_hdlr=gate3_hdlr,  exit_hdlr=None,   debug_info='Passing through gate 3...')
        gymkhana_challenge_node.fsm.add_state(state_name='Exit Channel',  state_hdlr=exit_hdlr,   exit_hdlr=None,   debug_info='Exiting channel...')
        gymkhana_challenge_node.fsm.add_state(state_name='Find Pinger',   state_hdlr=pinger_hdlr, exit_hdlr=None,   debug_info='Finding pinger...')
        gymkhana_challenge_node.fsm.add_state(state_name='Hold Position', state_hdlr=park_hdlr,   exit_hdlr=None,   debug_info='Station-keeping over pinger...')

        # Wait for valid messages to ensure proper state initialization
        rospy.wait_for_message('/vrx/task/info', Task)
        rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
        rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
        rospy.wait_for_message('/wamv/sensors/pingers/pinger/range_bearing', RangeBearing)

        while not rospy.is_shutdown():
            gymkhana_challenge_node.fsm.run()
            rate = rospy.Rate(10)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
