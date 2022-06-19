#!/usr/bin/env python3

import math
import numpy
import rospy
import actionlib
from std_msgs.msg import Empty
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, Twist # !!! Pose.orientation NOT used as a quaternion !!!
from sensor_msgs.msg import NavSatFix, Imu, Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from geographic_msgs.msg import GeoPoseStamped
from vrx_gazebo.msg import Task
from singaboat_vrx.msg import MissionGoal, MissionAction
from singaboat_vrx.cfg import ScanDockDeliverConfig
from singaboat_vrx.perception_module import ColorSequenceDetector, BayDetector
from singaboat_vrx.planning_module import Mission
from singaboat_vrx.control_module import PolarController
from singaboat_vrx.common_utilities import quaternion_to_euler, euler_to_quaternion, cartesian_to_polar, euclidean_distance, normalize_angle, local_to_global_tf, gps_to_enu, enu_to_gps

################################################################################

# Reference list containing interested colors
interested_colors = ["Red", "Yellow", "Blue", "Green"]

################################################################################

class ScanDockDeliver:
    def __init__(self):
        # Initialize scan-dock-deliver
        self.cv_bridge    = CvBridge() # CvBridge object
        self.image        = None # Image
        self.docking_time = None # Docking time
        self.asv_geopose  = Mission.Pose(gps_lat=0, gps_lon=0) # ASV pose in GPS coordinates
        self.asv_cartpose = Pose() # ASV pose in cartesian (ENU) coordinates
        self.target_pose  = None # Target bay pose
        self.near_dock    = False # Whether ASV is near the dock (in front of bays)
        self.near_bay     = False # Whether ASV is very close to the target bay
        self.inside_bay   = False # Whether ASV is inside the target bay
        self.config       = {} # Scan-dock-deliver configuration
        # ROS infrastructure
        self.cmd_vel_pub    = None
        self.shooter_pub    = None
        self.dyn_reconf_srv = None

    def gps_callback(self, msg):
        if self.asv_geopose.heading is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        # Convert to ENU coordinates for offset correction
        x, y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        x += self.gps_offset * math.cos(self.asv_geopose.heading)
        y += self.gps_offset * math.sin(self.asv_geopose.heading)
        # Update `asv_cartpose`
        self.asv_cartpose.position.x = x
        self.asv_cartpose.position.y = y
        # Convert back to GPS coordinates
        self.asv_geopose.gps_lat, self.asv_geopose.gps_lon, _ = enu_to_gps(x, y)

    def imu_callback(self, msg):
        self.asv_geopose.heading = self.asv_cartpose.orientation.z = quaternion_to_euler(msg.orientation)[2]

    def camera_callback(self, msg):
        # Try converting ROS Image message to OpenCV image
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(error)
            print()

    def geopose_stamped(self, pose, wp_mode="PASS"):
        '''
        Convert custom geopose to ROS GeoPoseStamped message.

        :param pose   : Mission.Pose object
        :param wp_mode: Waypoint mode

        :return geopose_stamped: geographic_msgs.GeoPoseStamped object
        '''
        geopose_stamped = GeoPoseStamped()
        geopose_stamped.header.stamp = rospy.Time.now()
        geopose_stamped.header.frame_id = wp_mode
        geopose_stamped.pose.position.latitude = pose.gps_lat
        geopose_stamped.pose.position.longitude = pose.gps_lon
        q = euler_to_quaternion(0, 0, pose.heading)
        geopose_stamped.pose.orientation.x = q[0]
        geopose_stamped.pose.orientation.y = q[1]
        geopose_stamped.pose.orientation.z = q[2]
        geopose_stamped.pose.orientation.w = q[3]
        return geopose_stamped

    def navigate(self, goal_pos_x, goal_pos_y, goal_rot_z, goal_mode="PASS"):
        '''
        Create a path from the current position of ASV to the given waypoint, and send
        the path from this node (action client) to the mission manager (action server).

        :param goal_pos_x: X-coordinate of goal pose
        :param goal_pos_y: Y-coordinate of goal pose
        :param goal_rot_z: Orientation of goal pose

        :return: None
        '''
        goal_pose = Mission.Pose(enu_x=goal_pos_x, enu_y=goal_pos_y, heading=goal_rot_z)
        goal_geopose_stamped = self.geopose_stamped(goal_pose, wp_mode=goal_mode)
        asv_geopose_stamped = self.geopose_stamped(self.asv_geopose)
        path = [asv_geopose_stamped, goal_geopose_stamped]
        goal = MissionGoal(mission=path)
        self.action_client.send_goal(goal)

    def scan(self):
        '''
        Command the ASV to wait at its current position and scan the environment. Send
        this command from this node (action client) to the mission manager (action server).

        :param: None

        :return: None
        '''
        asv_geopose_stamped = self.geopose_stamped(self.asv_geopose, wp_mode="SCAN")
        command = [asv_geopose_stamped]
        goal = MissionGoal(mission=command)
        self.action_client.send_goal(goal)

    def drive_forward(self, distance=2.0):
        '''
        Drive the ASV forward (in the direction of its current heading) by a certain amount of distance.

        :param distance: Amount of distance to move forward

        :return: None
        '''
        asv_pos_x, asv_pos_y, asv_rot_z = self.asv_cartpose.position.x, self.asv_cartpose.position.y, self.asv_cartpose.orientation.z
        goal_pos_x, goal_pos_y, goal_rot_z = asv_pos_x + distance * math.cos(asv_rot_z), asv_pos_y + distance * math.sin(asv_rot_z), asv_rot_z
        self.navigate(goal_pos_x, goal_pos_y, goal_rot_z)

    def caution_waypoint(self, goal_pos_x, goal_pos_y, goal_rot_z, distance=2.0):
        '''
        Create a caution waypoint by shifting the goal pose a certain amount of distance towards the WAM-V
        to allow room for adjustments through temporary station-keeping.

        :param distance: Amount of distance to shift the goal pose

        :return cwp_pos_x, cwp_pos_y, cwp_rot_z: Pose of the caution waypoint (CWP)
        '''
        cwp_pos_x, cwp_pos_y, cwp_rot_z = goal_pos_x - distance * math.cos(goal_rot_z), goal_pos_y - distance * math.sin(goal_rot_z), goal_rot_z
        return cwp_pos_x, cwp_pos_y, cwp_rot_z

    def update_target_pose(self, target_pos_x, target_pos_y, target_rot_z):
        '''
        Update target pose by filtering out potential abrupt changes.

        :param target_pos_x: X-coordinate of target pose
        :param target_pos_y: Y-coordinate of target pose
        :param target_rot_z: Orientation of target pose

        :return: None
        '''
        assert self.target_pose is not None
        if self.debug:
            print("Previous Target Pose: {} m, {} m, {}°".format(self.target_pose.position.x, self.target_pose.position.y, numpy.rad2deg(self.target_pose.orientation.z)))
            print("Requested Target Pose: {} m, {} m, {}°".format(target_pos_x, target_pos_y, numpy.rad2deg(target_rot_z)))
            print()
        new_target_pose = Pose()
        new_target_pose.position.x = target_pos_x
        new_target_pose.position.y = target_pos_y
        new_target_pose.orientation.z = target_rot_z
        dist = euclidean_distance(self.target_pose, new_target_pose)
        angle_diff = numpy.rad2deg(abs(normalize_angle(self.target_pose.orientation.z - new_target_pose.orientation.z)))
        if dist < 10 and angle_diff < 20:
            if self.debug:
                print("Updating the target pose...")
                print()
            self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z = target_pos_x, target_pos_y, target_rot_z
        else:
            if self.debug:
                print("Requested target pose is very different from the previous target pose.")
                print("Not updating the target pose...")
                print()
        if self.debug:
            print("Current Target Pose: {} m, {} m, {}°".format(self.target_pose.position.x, self.target_pose.position.y, numpy.rad2deg(self.target_pose.orientation.z)))
            print()

    def reached_near_bay(self, dist_tol=1.0, angle_tol=20.0):
        '''
        Check weather the ASV has reached near (within the prescribed tolerance of) the target bay.

        :param dist_tol : Distance tolerance in meters
        :param angle_tol: Orientation tolerance in degrees

        :return result: Boolean flag determining weather the ASV has reached near the target bay
        '''
        assert self.target_pose is not None
        dist = euclidean_distance(self.asv_cartpose, self.target_pose)
        angle = abs(normalize_angle(self.asv_cartpose.orientation.z - self.target_pose.orientation.z))
        if self.debug:
            print("ASV Pose:")
            print(self.asv_cartpose)
            print("Target Pose:")
            print(self.target_pose)
            print()
            print("Position Error: {} m".format(dist))
            print("Orientation Error: {}°".format(numpy.rad2deg(angle)))
            print()
        result = dist < dist_tol and angle < numpy.deg2rad(angle_tol)
        return result

    def dock(self, target_bay_pose=None, caution=True):
        '''
        Drive the ASV near the dock (i.e. in front of the docking bays) using Dubin's path tracker,
        approach the target bay using polar controller, and perform docking operation with caution
        using Dubin's path tracker.

        :param target_bay_pose: Target bay pose w.r.t. WAM-V frame
        :param caution        : Flag to enable/disable creation of caution waypoint(s)

        :return: None
        '''
        if target_bay_pose is not None:
            assert len(target_bay_pose) == 3
            # Current pose of the ASV
            asv_pos_x, asv_pos_y, asv_rot_z = self.asv_cartpose.position.x, self.asv_cartpose.position.y, self.asv_cartpose.orientation.z
            # Bay pose w.r.t. WAM-V frame
            bay_pos_x, bay_pos_y, bay_rot_z = target_bay_pose[0] + self.lidar_offset, target_bay_pose[1], target_bay_pose[2]
            # Add caution waypoint
            cwp_pos_x, cwp_pos_y, cwp_rot_z = self.caution_waypoint(bay_pos_x, bay_pos_y, bay_rot_z, distance=(5 if caution else 0))
            # Transform target pose to global frame
            target_pos_x, target_pos_y, target_rot_z = local_to_global_tf(asv_pos_x, asv_pos_y, asv_rot_z, cwp_pos_x, cwp_pos_y, cwp_rot_z)
            # Update target pose
            if self.target_pose is None:
                self.target_pose = Pose()
                self.target_pose.position.x = target_pos_x
                self.target_pose.position.y = target_pos_y
                self.target_pose.orientation.z = target_rot_z
            else:
                self.update_target_pose(target_pos_x, target_pos_y, target_rot_z) # Update target pose by filtering out potential abrupt changes
        # Move ASV near the dock (i.e. in front of the docking bays)
        if not self.near_dock:
            target_pos_x, target_pos_y, target_rot_z = self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z
            cwp_pos_x, cwp_pos_y, cwp_rot_z = self.caution_waypoint(target_pos_x, target_pos_y, target_rot_z, distance=30)
            self.navigate(cwp_pos_x, cwp_pos_y, cwp_rot_z, goal_mode="HALT")
            self.action_client.wait_for_result(timeout=rospy.Duration(30))
            self.action_client.cancel_all_goals()
            self.near_dock = True
        # Move ASV near the target docking bay and perform docking operation
        else:
            if not self.near_bay:
                self.near_bay = self.reached_near_bay(dist_tol=16, angle_tol=20)
            # If near the target docking bay, perform docking operation with caution
            if self.near_bay:
                if not self.inside_bay: # Check whether docking has been completed
                    self.inside_bay = self.reached_near_bay(dist_tol=4, angle_tol=20)
                    if self.inside_bay:
                        print("Docking successful!")
                        print()
                target_pos_x, target_pos_y, target_rot_z = self.caution_waypoint(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z, distance=self.caution_distance)
                print("ASV has reached near the target bay, attempting to dock...")
                if self.debug:
                    print("Target Bay Pose: {} m, {} m, {} rad".format(target_pos_x, target_pos_y, target_rot_z))
                print()
                self.navigate(target_pos_x, target_pos_y, target_rot_z, goal_mode="PARK")
                # Wait (station-keeping) for 5 seconds at each intermediate (caution) waypoint for safer docking
                self.action_client.wait_for_result(timeout=rospy.Duration(5))
                if self.debug:
                    print("Successfully passed caution waypoint!")
                    print()
                self.caution_distance -= self.caution_step  # Set next intermediate (caution) waypoint 3 meters forward
                self.caution_distance = max(0, self.caution_distance) # Do not exceed target pose
            # If not near the target docking bay, try to reach the target docking bay with higher speed (using polar controller)
            else:
                rho, alpha, beta = cartesian_to_polar(self.asv_cartpose, self.target_pose) # Convert target pose to polar coordinates
                # Generate and publish `cmd_vel` message
                cmd_vel_msg = self.polar_controller.control(rho, alpha, beta) # Polar controller output
                print("ASV has reached near the dock, approaching the target bay...")
                if self.debug:
                    print("rho: {}, alpha: {}, beta: {}, lin_vel_x: {}, ang_vel_z: {}".format(rho, alpha, beta, cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))
                print()
                self.cmd_vel_pub.publish(cmd_vel_msg)

    def deliver(self):
        '''
        Drive the ASV to the right corner of the docking bay and then shoot the projectiles.

        :param: None

        :return: None
        '''
        if self.docking_time is None:
            self.docking_time = rospy.get_time() # Update `docking_time` only once after docking
        if self.debug:
            print("Docking Time: {}".format(self.docking_time))
            print()
        # Drive the ASV to the right corner of the docking bay
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.67
        cmd_vel_msg.linear.y = -1.67
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        if self.debug:
            print("Elapsed Time: {}".format(rospy.get_time() - self.docking_time))
            print()
        if rospy.get_time() - self.docking_time >= self.delivery_delay:
            # Keep the ASV at the right corner of the docking bay
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 1.67
            cmd_vel_msg.linear.y = -1.67
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            # Shoot the projectiles
            print("Shooting projectiles...")
            print()
            self.shooter_pub.publish()
            self.ball_balance -= 1

    def exit(self):
        '''
        Safely exit the docking bay by driving the ASV backward.

        :param: None

        :return: None
        '''
        print("Exiting the docking bay...")
        print()
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -2.23 # Drive the ASV backward
        rate = rospy.Rate(10)
        while end_time - start_time < 10:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            end_time = rospy.get_time()

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.polar_controller = PolarController(config['k_rho'], config['k_alpha'], config['k_beta'], config['max_lin_vel'], config['max_ang_vel']) # Polar controller
        self.gps_offset       = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.lidar_offset     = config['lidar_offset'] # LIDAR offset w.r.t. WAM-V along X-axis
        self.caution_distance = config['caution_distance'] # Caution distance from target pose
        self.caution_step     = config['caution_step'] # Step size to decrement caution distance from target pose
        self.ball_balance     = config['ball_balance'] # Number of balls yet to be shot from ASV
        self.delivery_delay   = config['delivery_delay'] # Time delay in seconds from docking to delivery
        self.debug            = config['debug'] # Flag to enable/disable debug messages
        self.config           = config
        return config

################################################################################

if __name__ == "__main__":
    rospy.init_node('singaboat_scan_dock_deliver', anonymous = True)

    # ScanDockDeliver class instance
    scan_dock_deliver_node = ScanDockDeliver()

    # Dynamic reconfigure server
    scan_dock_deliver_node.dyn_reconf_srv = Server(ScanDockDeliverConfig, scan_dock_deliver_node.config_callback)

    # Color sequence and docking bay detectors
    scan_dock_deliver_node.color_sequence_detector = ColorSequenceDetector(interested_colors, debug=scan_dock_deliver_node.debug) # ColorSequenceDetector class instance
    scan_dock_deliver_node.bay_detector = BayDetector(interested_colors, debug=scan_dock_deliver_node.debug) # BayDetector class instance

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, scan_dock_deliver_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, scan_dock_deliver_node.imu_callback)
    rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, scan_dock_deliver_node.camera_callback)
    rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, scan_dock_deliver_node.color_sequence_detector.camera_callback)
    rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, scan_dock_deliver_node.bay_detector.camera_callback)
    rospy.Subscriber('/wamv/sensors/lidars/lidar/points', PointCloud2, scan_dock_deliver_node.bay_detector.dock_detector.lidar_callback)

    # Publishers
    scan_dock_deliver_node.cmd_vel_pub = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size=1)
    scan_dock_deliver_node.shooter_pub = rospy.Publisher('/wamv/shooters/ball_shooter/fire', Empty, queue_size=1)

    # Action client
    scan_dock_deliver_node.action_client = actionlib.SimpleActionClient('singaboat_mission_manager', MissionAction)
    scan_dock_deliver_node.action_client.wait_for_server()

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/wamv/sensors/cameras/camera/image_raw', Image)
    rospy.wait_for_message('/wamv/sensors/lidars/lidar/points', PointCloud2)

    try:
        rate = rospy.Rate(20)

        # Initialization
        while scan_dock_deliver_node.image is None:
            if scan_dock_deliver_node.debug:
                print()
                print("Initializing...")
                print()
            rospy.sleep(rospy.Duration(secs=0, nsecs=1000*1000*500))

        # 1. SCAN
        scan_dock_deliver_node.scan() # Scan and decode color sequence of the light buoy
        print()
        print("Scanning light buoy color sequence...")
        print()
        color, shape = scan_dock_deliver_node.color_sequence_detector.detect()
        scan_dock_deliver_node.bay_detector.target_bay_symbol(target_color=color, target_shape=shape)
        while not rospy.is_shutdown():
            rate.sleep()
            target_bay_pose = None
            if not scan_dock_deliver_node.near_dock:
                target_bay_pose, all_bay_poses = scan_dock_deliver_node.bay_detector.detect()
            theta, plane_cloud = scan_dock_deliver_node.bay_detector.plane_angle_and_cloud()
            if theta is not None:
                if scan_dock_deliver_node.debug:
                    print("Dock plane angle w.r.t. WAM-V frame: {}°".format(numpy.rad2deg(theta)))
                    print()
                if scan_dock_deliver_node.near_dock:
                    target_bay_pose, all_bay_poses = scan_dock_deliver_node.bay_detector.detect(cloud=plane_cloud)
                if target_bay_pose is not None:
                    assert len(target_bay_pose) == 3
                    if scan_dock_deliver_node.debug:
                        print("Dock plane angle got updated from {}° to {}°".format(target_bay_pose[2], theta))
                        print()
                    target_bay_pose[2] = theta
            else:
                if scan_dock_deliver_node.debug:
                    print("Dock plane angle computation failed!")
                    print()

            # 2. DOCK
            if target_bay_pose is None and scan_dock_deliver_node.target_pose is None: # If bay detection fails, try moving forward
                if scan_dock_deliver_node.debug:
                    print("Try moving forward...")
                    print()
                scan_dock_deliver_node.drive_forward(distance=5)
                continue
            if not scan_dock_deliver_node.inside_bay: # If bay is detected successfully, try moving towards it and perform docking operation
                if scan_dock_deliver_node.debug:
                    print("Try moving towards the target bay...")
                    print()
                scan_dock_deliver_node.dock(target_bay_pose)

            # 3. DELIVER
            else: # If docking is successful, try shooting the projectiles
                scan_dock_deliver_node.action_client.cancel_all_goals()
                scan_dock_deliver_node.deliver() # Shoot projectiles
                if scan_dock_deliver_node.ball_balance == 0:
                    scan_dock_deliver_node.exit() # Exit the docking bay
                    break

    except rospy.ROSInterruptException:
        pass
