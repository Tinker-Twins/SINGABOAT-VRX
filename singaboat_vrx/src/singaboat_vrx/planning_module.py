#!/usr/bin/env python3

import numpy
import copy
import dubins
from singaboat_vrx.common_utilities import enu_to_gps, gps_to_enu

################################################################################

class FiniteStateMachine:
    '''
    Creates a finite state machine and provides tools to define the states and
    corresponding handlers that will implement the logic to transition between
    the states of the state machine.
    '''
    def __init__(self):
        self.current_state  = '' # Variable to keep track of the current state
        self.debug_info     = {} # Dictionary of information pertaining to all the defined states
        self.state_handlers = {} # Dictionary of state handlers for all the defined states
        self.exit_handlers  = {} # Dictionary of exit handlers for all the defined states

    def add_state(self, state_name, state_hdlr=None, exit_hdlr=None, start_state=False, end_state=False, debug_info=None):
        '''
        Add a new state to the finite state machine.

        :param state_name : Unique state name (string)
        :param state_hdlr : Callback to handle the function(s) while in this state
        :param exit_hdlr  : Callback to handle the function(s) before exiting this state
        :param start_state: Boolean flag to define this state as the start-state of the FSM
        :param end_state  : Boolean flag to define this state as the end-state of the FSM
        :param debug_info : Descriptive information of this state (for debugging purposes)

        :return: None
        '''
        if isinstance(state_name, str): # Sanity check
            self.state_handlers[state_name] = state_hdlr # Load state handlers
            self.exit_handlers[state_name] = exit_hdlr # Load exit handlers
            self.debug_info[state_name] = debug_info # Load debug information
            if start_state == True:
                self.current_state = state_name
        else:
            raise ValueError("State name must be a string.")

    def run(self):
        '''
        Run the finite state machine.
        Note: This method must be executed continuously to trigger the state handlers
        and state transition logic correctly.

        :param : None

        :return: None
        '''
        if self.debug_info[self.current_state] != None:
            print()
            print(self.debug_info[self.current_state]) # Print debug information
            print()
        state_hdlr = self.state_handlers[self.current_state] # Fetch state handler for the current state
        next_state = state_hdlr() # Execute the state handler (runs logic for the current state and returns the next state)
        if next_state != self.current_state:
            if self.exit_handlers[self.current_state] != None:
                self.exit_handlers[self.current_state]() # Execute the exit handler (if any)
        self.current_state = next_state # Transition to the next state

################################################################################

class Mission:
    '''
    `Mission` defines various child-classes needed for planning and executing a mission.
    '''
    def __init__(self):
        pass

    class Pose():
        '''
        `Pose` defines position (GPS and/or ENU coordinates) and orientation
        (heading). It is the building block of a higher level class `Waypoint`,
        which comprises `Pose` as one of its attributes.
        '''
        def __init__(self, gps_lat = -1, gps_lon = -1, enu_x = -1, enu_y = -1, heading = -1):
            self.gps_lat, self.gps_lon = gps_lat, gps_lon
            self.enu_x, self.enu_y = enu_x, enu_y
            self.heading = heading
            # Convert position coordinates to the complementary format for potential future use
            if enu_x != -1 and enu_y != -1:
                self.convert_to_gps()
            elif gps_lat != -1 and gps_lon != -1:
                self.convert_to_enu()
            else:
                raise ValueError("Provide at least one way to initialize (GPS and/or ENU coordinates).")

        def convert_to_gps(self):
            self.gps_lat, self.gps_lon, _ = enu_to_gps(self.enu_x, self.enu_y)

        def convert_to_enu(self):
            self.enu_x, self.enu_y, _ =  gps_to_enu(self.gps_lat, self.gps_lon)

    class Waypoint:
        '''
        `Waypoint` is a virtual object, which defines nvaigation instructions
        such as tracking mode and target pose. It is the building block of a
        higher level class `Path`, which is composed of several waypoints.
        ---------------
        Waypoint Modes:
        ---------------
        NONE = Not defined (used for initialization)
        PASS = Drive-through waypoint (used for general path tracking)
        HALT = Temporary station-keeping waypoint (used for accurate path tracking)
        PARK = Indefinite station-keeping waypoint (used for station-keeping)
        SCAN = Scanning waypoint (used for halting and scanning around)
        '''
        def __init__(self, wp_mode = "NONE", gps_lat = -1, gps_lon = -1, enu_x = -1, enu_y = -1, heading = 0):
            self.wp_mode = wp_mode
            self.pose = Mission.Pose(gps_lat, gps_lon, enu_x, enu_y, heading)

        def convert_to_enu(self):
            self.pose.convert_to_enu()
            return self

        def convert_to_gps(self):
            self.pose.convert_to_gps()
            return self

    class Path:
        '''
        `Path` is composed of several waypoints arranged in an orderly fashion,
        which together define a path to be tracked by the ASV.
        '''
        def __init__(self, waypoints = []):
            self.wps = []
            wps = waypoints
            for wp in wps:
                self.wps.append(Mission.Waypoint(wp_mode = wp['wp_mode'], gps_lat = wp['lat'], gps_lon = wp['lon'], heading = wp['heading']))

        def __getitem__(self, idx):
            return self.wps[idx]

################################################################################

class DubinsPath:
    '''
    Generates a Dubin's path.
    '''
    def __init__(self, mission, turn_radius=8.0, step_size=1.0):
        self.turning_radius = turn_radius # Turning radius in meters
        self.step_size = step_size  # Step size in meters
        self.path = [] # Dubins path
        path = []
        wp_idxs = [0]
        for idx, wp in enumerate(mission):
            pos_x = wp.convert_to_enu().pose.enu_x
            pos_y = wp.convert_to_enu().pose.enu_y
            rot_z = wp.pose.heading
            q1 = (pos_x, pos_y, rot_z)
            if idx > 0:
                if self.measure_distance(q0, q1) > 9*self.step_size:
                    tmp_path = dubins.shortest_path(q0, q1, self.turning_radius)
                    configurations, _ = tmp_path.sample_many(self.step_size)
                else:
                    configurations = [q0, q1]
                last = wp_idxs[-1]
                wp_idxs.append(last+len(configurations)+1)
                path.extend(configurations)
            q0 = q1
        cnt = 0
        for i, p in enumerate(path):
            if wp_idxs[cnt] != i:
                wp = Mission.Waypoint(enu_x = p[0], enu_y = p[1], heading = p[2])
                self.path.append(wp)
            else:
                wp = copy.deepcopy(mission[cnt])
                wp.pose.heading = wp.pose.heading
                self.path.append(wp)
                cnt += 1
        wp = copy.deepcopy(mission[cnt])
        wp.pose.heading = wp.pose.heading
        self.path.append(wp)

    def measure_distance(self, q0, q1):
        '''
        Compute straight line distance between two waypoints points.

        :param q0: First point (pose)
        :param q1: Second point (pose)

        :return dist: Distance between the two points
        '''
        q0 = numpy.array(q0)
        q1 = numpy.array(q1)
        dist = numpy.linalg.norm(q0[0:-1] - q1[0:-1])
        return dist

################################################################################

class PathPlanner:
    def __init__(self, mission, path_creator, handover_offset=5):
        self.mission           = mission # Mission
        self.path_creator      = path_creator # Path creator
        self.path_hdlr         = path_creator(mission) # Path creater object
        self.mission_complete  = False # Mission completion flag
        self.lap_count         = 1 # Number of times to repeat the mission
        self.lap_counter       = 0 # Lap counter
        self.original_path     = self.path_hdlr.path # Original path
        self.working_path      = self.original_path.copy() # Working path
        self.working_index     = 0 # Index of waypoint on the working path
        self.original_index    = 0 # Index of waypoint on the original path
        self.current_wp        = self.original_path[0] # Current waypoint
        self.next_wp           = self.original_path[0] # Next waypoint
        self.proj_heading      = 0 # Projected heading at the goal
        self.desired_heading   = 0 # Desired (go-to-goal) heading of the WAM-V
        self.cross_track_error = 0 # Cross-track error in meters
        self.beta_hat          = 0 # Integral term for computing the desired heading using ILOS guidance method
        self.look_ahead_dist   = 30 # Lookahead distance in meters
        self.min_wp_dist       = 25 # Minimum distance between two waypoints so as to allow the WAM-V to get on track
        self.max_deviation     = 10 # Maximum permissible deviation from path before triggering a replan
        self.handover_offset   = handover_offset # Distance before the end of the path to handover control of the WAMV-V

    def measure_distance(self, wp1, wp2):
        '''
        Compute straight line distance between two waypoints points.

        :param wp1: First waypoint (pose)
        :param wp2: Second waypoint (pose)

        :return dist: Distance between the two waypoints
        '''
        pos_x_1, pos_y_1 = wp1.pose.enu_x, wp1.pose.enu_y
        pos_x_2, pos_y_2 = wp2.pose.enu_x, wp2.pose.enu_y
        dist = numpy.sqrt((pos_x_1-pos_x_2)**2+(pos_y_1-pos_y_2)**2)
        return dist

    def distance_to_path(self, current_wp, path, current_index, opt_range=40):
        '''
        Compute distance to the closest point on a path, by optimizing within
        `opt_range` number of steps in either direction of `current_index`.

        :param current_wp   : Current pose of the WAM-V in the form of a waypoint
        :param path         : List containing ENU coordinates of all waypoints along the current path
        :param current_index: Index of the previous path ENU coordinate, which was goal along that path
        :param opt_range    : Range of optimization (how far along both directions of the path should the potential point be searched)

        :return distance    : Distance to the closest waypoint on the path in meters
        :return proj_heading: Projected heading at the given closest waypoint in radians
        :return path_index  : Index of the closest waypoint on the path
        '''
        dist = self.measure_distance(current_wp, path[current_index]) # Calculate distance to last point
        proj_index = min(len(path)-1,current_index+7) # Get a point ahead, without exceeding the path length
        proj_heading = path[proj_index].pose.heading # Pull out last goal yaw at last point
        path_index = current_index # Pull out last index
        # Optimization loop for each point within 40 steps along the path (based on the step size of the Dubin's path)
        for index in range(current_index - opt_range, current_index + opt_range):
            index = min(max(index, 0), len(path) - 1)
            temp_dist = self.measure_distance(current_wp, path[index]) # Calculate distance to newly iterated point
            if temp_dist < dist: # If this new point is closer to the WAM-V, switch to it
                dist = temp_dist # Save distance to the new point
                proj_heading = path[proj_index].pose.heading # Save goal heading at the new point
                path_index = index # Save index of the new point
        # Compute distance error sign change (to get a signed error)
        R = numpy.transpose(numpy.array([[numpy.cos(proj_heading), - numpy.sin(proj_heading)], [numpy.sin(proj_heading), numpy.cos(proj_heading)]])) # Get the rotation matrix
        x_e, y_e = numpy.dot(R, numpy.array([[(current_wp.pose.enu_x - path[path_index].pose.enu_x)], [(current_wp.pose.enu_y - path[path_index].pose.enu_y)]])) # Compute the distance error
        distance = y_e[0]
        return distance, proj_heading, path_index

    def ilos_guidance(self, cte, beta_hat, proj_heading, cur_speed, delta=6, gamma=0):
        '''
        Compute the integral line of sight (ILOS) guidance values to reduce the
        cross-track error, even in the presence of external disturbances.

        :param cte         : Cross-track error
        :param beta_hat    : Integral term for computing the desired heading
        :param proj_heading: Projected heading of the curve at the i-th position
        :param cur_speed   : Current speed of the WAM-V
        :param delta       : Lookahead distance
        :param gamma       : Coefficient used to scale up/down the integral term `beta_hat`

        :return desired_heading: Desired heading of the WAM-V
        :return beta_hat       : Accumulated `beta_hat` term after the one-step integration
        '''
        beta_hat = beta_hat + (cte*gamma*cur_speed*delta)/(numpy.sqrt(delta**2+(cte+delta*beta_hat)**2))
        desired_heading = numpy.arctan(-beta_hat-(cte/delta)) + proj_heading
        return desired_heading, beta_hat

    def replan(self, asv_pose, current_wp_idx, original_path):
        '''
        Generate a path containing a new route to get back on track,
        along with the remaining path from the original mission.

        :param asv_pose      : Current pose of the WAM-V in the form of a waypoint
        :param current_wp_idx: Index of the current target waypoint in the working path
        :param original_path : Original path that was created based on the initial mission

        :return recovery_path: Re-planned path
        '''
        target_idx = int(min(current_wp_idx + numpy.floor(self.look_ahead_dist/self.path_hdlr.step_size), len(original_path)-numpy.floor(self.min_wp_dist/self.path_hdlr.step_size)))
        start = asv_pose
        end = copy.deepcopy(original_path[target_idx])
        mission = Mission.Path(waypoints=[start, end])
        recovery_path = self.path_creator(mission).path
        recovery_path.extend(copy.deepcopy(original_path[target_idx:]))
        return recovery_path

    def update(self, current_wp, speed):
        '''
        Update path planner on-the-go to safely execute the mission and to account
        for any dynamic re-planning.
        Note: This function needs to be called at fixed time intervals.

        :param current_wp    : Current pose of the WAM-V in the form of a waypoint
        :param speed         : Current speed of the WAM-V

        :return mission_complete : Boolean flag indicating whether mission has been completed
        :return desired_heading  : Desired heading of the WAM-V
        :return cross_track_error: Cross-track error
        :return target_waypoint  : Target waypoint
        '''
        self.current_wp = current_wp
        if(self.current_wp.pose.gps_lat != 0):
            self.cross_track_error, self.proj_heading, self.working_index = self.distance_to_path(self.current_wp, self.working_path, self.working_index)
            _, _, self.original_index = self.distance_to_path(self.current_wp, self.original_path, self.original_index)
            # Check whether end of the path has been reached
            if self.working_index >= (len(self.working_path)-self.handover_offset):
                self.lap_counter = self.lap_counter + 1
                if self.lap_counter >= self.lap_count:
                    self.mission_complete = True
                    self.cross_track_error = 0 # Force the cross-track error to prevent any erratic behavior of the controller once the misssion has been completed
                    self.original_index = len(self.original_path) - 1
                    self.working_index = len(self.working_path) - 1
                else:
                    self.original_index = 1
                    self.working_index = 1
                    self.cross_track_error = 0
            # Replanning logic in case the WAM-V deviates too much (more than 10 m) from the closest point on the path
            if numpy.abs(self.cross_track_error) > self.max_deviation:
                self.working_path = self.replan(asv_pose = self.current_wp, current_wp_idx = self.original_index, original_path = self.original_path)
                self.working_index = 0 # Make sure to start from the first element of the augmented path
            # Update the planner
            self.desired_heading, self.beta_hat = self.ilos_guidance(cte = self.cross_track_error, beta_hat = self.beta_hat, proj_heading = self.proj_heading, cur_speed = speed)
            mission_complete, desired_heading, cross_track_error, target_waypoint = self.mission_complete, self.desired_heading, self.cross_track_error, self.original_path[self.original_index]
        return mission_complete, desired_heading, cross_track_error, target_waypoint
