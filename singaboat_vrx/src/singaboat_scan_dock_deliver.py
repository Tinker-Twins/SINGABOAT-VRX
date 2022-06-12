#!/usr/bin/env python3

import math
import numpy
import rospy
import actionlib
from std_msgs.msg import Empty
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import NavSatFix, Imu, Image
from cv_bridge import CvBridge, CvBridgeError
from geographic_msgs.msg import GeoPoseStamped
from vrx_gazebo.msg import Task
from singaboat_vrx.msg import MissionGoal, MissionAction
from singaboat_vrx.cfg import ScanDockDeliverConfig
from singaboat_vrx.perception_module import ColorSequenceDetector, BayDetector
from singaboat_vrx.planning_module import Mission
from singaboat_vrx.common_utilities import quaternion_to_euler, euler_to_quaternion, cartesian_to_polar, euclidean_distance, normalize_angle, polar_controller, local_to_global_tf, gps_to_enu, enu_to_gps

################################################################################

# Reference list containing interested colors
interested_colors = ["red", "yellow", "blue", "green"]

################################################################################

class ScanDockDeliver:
    def __init__(self, node_name):
        # Initialize scan-dock-deliver
        self.node_name = node_name # TODO: move away from here
        rospy.init_node(self.node_name, anonymous=True) # TODO: move away from here
        self.rate = rospy.Rate(5) # TODO: move away from here
        self.color_sequence_detector = ColorSequenceDetector(interested_colors) # ColorSequenceDetector object
        self.bay_detector = BayDetector(interested_colors) # BayDetector object
        self.target_pose = None  # Target bay pose
        self.near_dock = False  # Whether ASV is near the dock (in front of bays)
        self.near_bay = False  # Whether ASV is very close to the target bay
        self.inside_bay = False  # Whether ASV is inside the target bay
        self.next_shift_dist = 10  # Pose keeping distance difference to target pose
        self.cv_bridge = CvBridge() # CvBridge object
        self.image = None # Image
        self.docking_time = None # Docking time
        self.ball_balance = 4 # Ball balance
        self.asv_geopose = Mission.Pose(gps_lat=0, gps_lon=0) # ASV pose in GPS coordinates
        self.asv_cartpose = Pose() # ASV pose in cartesian (ENU) coordinates
        self.debug = False # Flag to enable/disable debug messages | TODO: move to config
        self.config = {} # Scan-dock-deliver configuration
        self.dyn_reconf_srv = Server(ScanDockDeliverConfig, self.config_callback) # TODO: move away from here

        # Subscribers
        rospy.Subscriber('/vrx/task/info', Task, self.task_callback, queue_size=1) # TODO: move away from here
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback, queue_size=1) # TODO: move away from here
        rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, self.imu_callback, queue_size=1) # TODO: move away from here
        rospy.Subscriber('/wamv/sensors/cameras/camera/image_raw', Image, self.camera_callback, queue_size=1) # TODO: move away from here

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size=1) # TODO: move away from here
        self.shooter_pub = rospy.Publisher('/wamv/shooters/ball_shooter/fire', Empty, queue_size=10) # TODO: move away from here

        # Action client
        self.action_client = actionlib.SimpleActionClient('singaboat_mission_manager', MissionAction) # TODO: move away from here
        self.action_client.wait_for_server() # TODO: move away from here

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

    def task_callback(self, data):
        self.task_info = data

    def to_ros_geopose(self, pose, wp_type="PASS"):
        '''
        Convert custom geopose to ROS GeoPoseStamped message

        :param pose   : Mission.Pose
        :param wp_type: Waypoint mode

        :return: geographic_msgs.GeoPoseStamped
        '''
        geopose = GeoPoseStamped()
        geopose.header.stamp = rospy.Time.now()
        geopose.header.frame_id = wp_type
        geopose.pose.position.latitude = pose.gps_lat
        geopose.pose.position.longitude = pose.gps_lon
        q = euler_to_quaternion(0, 0, pose.heading)
        geopose.pose.orientation.x = q[0]
        geopose.pose.orientation.y = q[1]
        geopose.pose.orientation.z = q[2]
        geopose.pose.orientation.w = q[3]
        return geopose

    def send_goal(self, xg, yg, thetag, goal_type="PASS"):
        goal_pose = Mission.Pose(enu_x=xg, enu_y=yg, heading=thetag)
        goal_geopose_stamped = self.to_ros_geopose(goal_pose, wp_type=goal_type)
        asv_geopose_stamped = self.to_ros_geopose(self.asv_geopose)
        goals = MissionGoal(mission=[asv_geopose_stamped, goal_geopose_stamped])
        self.action_client.send_goal(goals)

    def stay_still(self):
        asv_geopose_stamped = self.to_ros_geopose(self.asv_geopose, wp_type="PLAN")
        goals = MissionGoal(mission=[asv_geopose_stamped])
        self.action_client.send_goal(goals)

    def move_forward(self, distance=2.0):
        x, y, theta = self.asv_cartpose.position.x, self.asv_cartpose.position.y, self.asv_cartpose.orientation.z
        xg, yg, thetag = x + distance * math.cos(theta), y + distance * math.sin(theta), theta
        self.send_goal(xg, yg, thetag)

    def get_shifted_target_pose(self, x, y, theta, distance=2.0):
        x, y, theta = x - distance * math.cos(theta), y - distance * math.sin(theta), theta
        return x, y, theta

    def move_to_target(self, target_bay=None, do_shift=True):
        '''
        Move in front of target bay with Dubin's path tracker, dock into the bay with local polar controller
        Note: Suffix `r` means pose in robot (WAM-V) or reference frame, suffix `g` means pose in global or ENU frame

        :param target_bay: pose in robot frame
        :param do_shift: whether to shift target pose towards robot to make room for station keeping

        :return:
        '''
        if target_bay is not None:
            assert len(target_bay) == 3
            # Transform target bay pose to global frame
            x, y, theta = self.asv_cartpose.position.x, self.asv_cartpose.position.y, self.asv_cartpose.orientation.z
            # Update shifted target pose in robot frame for visualization
            xr, yr, thetar = target_bay[0] + 0.7, target_bay[1], target_bay[2]
            xrs, yrs, thetars = self.get_shifted_target_pose(xr, yr, thetar, distance=(5 if do_shift else 0))
            xg, yg, thetag = local_to_global_tf(x, y, theta, xrs, yrs, thetars)
            if self.target_pose is None:
                self.target_pose = Pose()
                self.target_pose.position.x = xg
                self.target_pose.position.y = yg
                self.target_pose.orientation.z = thetag
            else:
                self.update_target_pose(xg, yg, thetag) # Update target pose
        if not self.near_dock: # Move ASV near the dock (i.e. in front of the docking bays)
            xg, yg, thetag = self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z
            xm, ym, thetam = self.get_shifted_target_pose(xg, yg, thetag, distance=30)
            self.send_goal(xm, ym, thetam, goal_type="HALT")
            self.action_client.wait_for_result(timeout=rospy.Duration(30))
            res = self.action_client.get_result()
            print("ASV is approaching the dock...")
            if self.debug:
                print("Dock Waypoint Action Result:".format(res))
            print()
            self.action_client.cancel_all_goals()
            self.near_dock = True
        else: # Move ASV near the docking bay
            if not self.near_bay:
                self.near_bay = self.reached_bay(max_dist=16, max_angle_diff=20)
            if self.near_bay:
                if not self.inside_bay: # Move ASV inside the docking bay
                    self.inside_bay = self.reached_bay(max_dist=3.99, max_angle_diff=35)
                xg, yg, thetag = self.get_shifted_target_pose(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z, distance=self.next_shift_dist)
                print("ASV has reached near the target bay, attempting to dock...")
                if self.debug:
                    print("Target Bay Pose: {} m, {} m, {} rad".format(xg, yg, thetag))
                print()
                self.send_goal(xg, yg, thetag, goal_type="PARK")
                # Wait (station-keeping) for 5 seconds at each intermediate waypoint for safer docking
                self.action_client.wait_for_result(timeout=rospy.Duration(5))
                if self.debug:
                    print("Station-keeping for {} meter shift finished.".format(self.next_shift_dist))
                    print()
                self.next_shift_dist -= 3  # Set next station-keeping pose 3 meters forward
                self.next_shift_dist = max(0, self.next_shift_dist) # Do not exceed target pose
            else: # Try to reach target bay with larger speed (using polar controller)
                rho, alpha, beta = cartesian_to_polar(self.asv_cartpose, self.target_pose)
                cmd_vel_msg = polar_controller(rho, alpha, beta, kr=0.2, ka=3.6, kb=0.1, max_linear_x=2.23, max_angular_z=0.89) # TODO: import PolarController object from control_module
                print("ASV has reached near the dock, approaching the target bay...")
                if self.debug:
                    print("rho: {}, alpha: {}, beta: {}, lin_vel_x: {}, ang_vel_z: {}".format(rho, alpha, beta, cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))
                print()
                self.cmd_vel_pub.publish(cmd_vel_msg)

    def reached_bay(self, max_dist=1.0, max_angle_diff=20.0):
        '''
        Check weather the ASV has reached near the target bay

        :param max_dist      : Distance threshold in meters
        :param max_angle_diff: Orientation threshold in degrees

        :return result: Boolean flag determining weather the ASV has reached near the target bay
        '''
        assert self.target_pose is not None
        dist = euclidean_distance(self.asv_cartpose, self.target_pose)
        theta = abs(normalize_angle(self.asv_cartpose.orientation.z - self.target_pose.orientation.z))
        if self.debug:
            print("ASV Pose:")
            print(self.asv_cartpose)
            print("Target Bay Pose:")
            print(self.target_pose)
            print()
            print("Driving to the target bay...")
            print("Position Error: {} m".format(dist))
            print("Orientation Error: {}°".format(numpy.rad2deg(theta)))
            print()
        return dist < max_dist and theta < numpy.deg2rad(max_angle_diff)

    def update_target_pose(self, xg, yg, thetag):
        '''
        Filter potential abrupt change of target pose to remove noise.

        :param xg: Goal position X-coordinate
        :param yg: Goal position Y-coordinate
        :param thetag: Goal orientation

        :return: None
        '''
        assert self.target_pose is not None
        if self.debug:
            print("Previous Bay Pose: {} m, {} m, {}°".format(self.target_pose.position.x, self.target_pose.position.y, numpy.rad2deg(self.target_pose.orientation.z)))
            print("Alternate Bay Pose: {} m, {} m, {}°".format(xg, yg, numpy.rad2deg(thetag)))
            print()
        new_target_pose = Pose()
        new_target_pose.position.x = xg
        new_target_pose.position.y = yg
        new_target_pose.orientation.z = thetag
        dist = euclidean_distance(self.target_pose, new_target_pose)
        angle_diff = numpy.rad2deg(abs(normalize_angle(self.target_pose.orientation.z - thetag)))
        if dist < 10 and angle_diff < 20:
            xn, yn, thetan = xg, yg, thetag
            self.target_pose.position.x, self.target_pose.position.y, self.target_pose.orientation.z = xn, yn, thetan
        else:
            if self.debug:
                print("New bay pose is far from the previous bay pose.")
                print()
        if self.debug:
            print("Updated Bay Pose: {} m, {} m, {}°".format(self.target_pose.position.x, self.target_pose.position.y, numpy.rad2deg(self.target_pose.orientation.z)))
            print()

    def shoot_balls(self):
        if self.docking_time is None:
            self.docking_time = rospy.get_time() # Update docking_time only once after docking
        if self.debug:
            print("Docking Time: {}".format(self.docking_time))
            print()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.67
        cmd_vel_msg.linear.y = 1.67
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        if self.debug:
            print("Elapsed Time: {}".format(rospy.get_time() - self.docking_time))
            print()
        if rospy.get_time() - self.docking_time >= 7:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 1.67
            cmd_vel_msg.linear.y = 1.67
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            print("Shooting balls...")
            print()
            self.shooter_pub.publish()
            self.ball_balance -= 1

    def exit_bay(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -2.23
        r = rospy.Rate(10)
        while end_time - start_time < 10:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            r.sleep()
            end_time = rospy.get_time()

    def run(self):
        # Initialization
        while self.image is None:
            if self.debug:
                print("Initializing...")
                print()
            rospy.sleep(rospy.Duration(secs=0, nsecs=1000*1000*500))
        if self.debug:
            print("ASV has all the sensory data ready!")
            print()
        # Detect and decode color sequence from the light buoy
        self.stay_still()
        print()
        print("Detecting light buoy color sequence...")
        if self.debug:
            print("Keeping the still for better color sequence detection...")
        print()
        color, shape = self.color_sequence_detector.detect()
        self.bay_detector.set_target_color_shape(target_color=color, target_shape=shape)
        while not rospy.is_shutdown():
            self.rate.sleep()
            bay_pose = None
            if not self.near_dock:
                bay_pose, all_bay_poses = self.bay_detector.detect()
            theta, plane_cloud = self.bay_detector.get_plane_theta_and_cloud()
            if theta is not None:
                if self.debug:
                    print("Plane theta in WAM-V frame: {}°".format(numpy.rad2deg(theta)))
                    print()
                if self.near_dock:
                    bay_pose, all_bay_poses = self.bay_detector.detect(cloud=plane_cloud)

                if bay_pose is not None:
                    assert len(bay_pose) == 3
                    if self.debug:
                        print("Plane theta got updated from {}° to {}°".format(bay_pose[2], theta))
                        print()
                    bay_pose[2] = theta
            else:
                if self.debug:
                    print("Plane theta computation failed!")
                    print()
            # If bay detection fails, try moving forward
            if bay_pose is None and self.target_pose is None:
                if self.debug:
                    print("Try moving forward...")
                    print()
                self.move_forward(distance=5)
                continue
            # If bay is detected successfully, try moving towards it
            if not self.inside_bay:
                if self.debug:
                    print("Try moving towards the target bay...")
                    print()
                self.move_to_target(bay_pose)
            # If docking is successful, try shooting balls
            else:
                print("Docking successful!")
                print()
                self.action_client.cancel_all_goals()
                self.shoot_balls() # Shoot balls
                if self.ball_balance == 0:
                    print("Exiting the docking bay...")
                    print()
                    self.exit_bay() # Exit the docking bay
                    break

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.config     = config
        return config

################################################################################

if __name__ == "__main__":
    scan_dock_deliver_node = ScanDockDeliver(node_name='ScanDockDeliver')
    try:
        scan_dock_deliver_node.run()
    except rospy.ROSInterruptException:
        pass
