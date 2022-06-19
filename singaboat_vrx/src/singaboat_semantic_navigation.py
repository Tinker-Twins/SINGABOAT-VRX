#!/usr/bin/env python3

import math
import numpy
import rospy
import tf
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Pose, Twist # !!! Pose.orientation NOT used as a quaternion !!!
from dynamic_reconfigure.server import Server
from singaboat_vrx.cfg import SemanticNavigationConfig
from singaboat_vrx.control_module import PolarController
from singaboat_vrx.common_utilities import gps_to_enu, quaternion_to_euler, normalize_angle, angle_within_half_plane_range, euclidean_distance, cartesian_to_polar

################################################################################

# Reference dictionaries storing attributes of each animal
ANIMAL_ACTION_DICT = {"crocodile": "circumvent", "platypus": "circle_cw", "turtle": "circle_ccw"}
ANIMAL_IDEAL_RADIUS_DICT = {"crocodile": 14, "platypus": 2, "turtle": 2}
ANIMAL_CRITICAL_RADIUS_DICT = {"crocodile": 11, "platypus": 1, "turtle": 1}
ANIMAL_STATUS_DICT = {"crocodile": True, "platypus": False, "turtle": False}

################################################################################

class Animal:
    '''
    Defines animals and their attributes.
    '''
    def __init__(self, name, pose, index):
        self.name            = name # Animal name
        self.pose            = pose # Animal pose
        self.index           = index # Animal index
        self.action          = ANIMAL_ACTION_DICT[self.name] # Action corresponding to a particular animal
        self.radius_ideal    = ANIMAL_IDEAL_RADIUS_DICT[self.name] # Ideal radius corresponding to a particular animal
        self.radius_critical = ANIMAL_CRITICAL_RADIUS_DICT[self.name] # Critical radius corresponding to a particular animal
        self.radius_actual   = self.radius_ideal # Actual radius corresponding to a particular animal
        self.asv_approaching = True # Boolean flag to check whether the ASV is approaching a particular animal
        self.asv_circling    = False # Boolean flag to check whether the ASV is circling a particular animal
        self.done            = ANIMAL_STATUS_DICT[self.name] # Boolean flag to check whether a particular animal has been navigated semantically

################################################################################

class CircleTracker:
    '''
    Tracks circling progress by accumulating yaw measurements.
    '''
    def __init__(self, yaw=0):
        self.cur_yaw = yaw
        self.acc_yaw = 0

    def update(self, yaw):
        self.acc_yaw += normalize_angle(yaw - self.cur_yaw)
        self.cur_yaw = yaw

    def reset(self, yaw):
        self.cur_yaw = yaw
        self.acc_yaw = 0

################################################################################

class SemanticNavigation:
    def __init__(self):
        # Initialize semantic navigation
        self.asv_pose       = Pose() # Current ASV pose in global frame
        self.animals        = [] # List to store all the animals
        self.animals_count  = 0 # Total count of all the animals
        self.circle_tracker = CircleTracker() # CircleTracker class instance
        self.init_debug     = True # Flag to enable/disable initial debug messages
        self.config         = {} # Semantic navigation configuration
        # ROS infrastructure
        self.tf_broadcaster       = None
        self.animal_model_viz_msg = Marker()
        self.animal_label_viz_msg = Marker()
        self.animal_model_viz_pub = None
        self.animal_label_viz_pub = None
        self.cmd_vel_msg          = None
        self.cmd_vel_pub          = None
        self.dyn_reconf_srv       = None

    def gps_callback(self, msg):
        if self.asv_pose.orientation.z is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        pos_x, pos_y, pos_z = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        pos_x += self.gps_offset * math.cos(self.asv_pose.orientation.z)
        pos_y += self.gps_offset * math.sin(self.asv_pose.orientation.z)
        self.asv_pose.position.x = pos_x
        self.asv_pose.position.y = pos_y
        self.cur_position = numpy.array([pos_x, pos_y, pos_z])

    def imu_callback(self, msg):
        self.asv_pose.orientation.z = quaternion_to_euler(msg.orientation)[2]
        self.cur_rotation = msg.orientation

    def wildlife_callback(self, msg):
        for index, animal in enumerate(msg.poses):
            name = animal.header.frame_id
            lat = animal.pose.position.latitude
            lon = animal.pose.position.longitude
            quat = animal.pose.orientation
            pose = Pose()
            # Update animal poses only after all animals are detected
            if self.animals_count:
                pose.position.x, pose.position.y, pose.position.z = gps_to_enu(lat, lon) # Position of animals in Z axis is not considered for ASV control (used for visualization only)
                pose.orientation = quat # Orientation of animals is not considered for ASV control (used for visualization only)
                self.animals[index].pose = pose # Replace with ENU pose
            else:
                self.animals.append(Animal(name, pose, index)) # Append animals to the list
        self.animals_count = len(self.animals) # Total count of all the animals

    def next_animal_index(self):
        '''
        Choose the nearest safe animal (platypus/turtle) to navigate towards.

        :param: None

        :return index: Index of the nearest safe animal (platypus/turtle)
        '''
        index = None # Initialize animal index
        min_dist = 1000000 # Initialize minimum distance to the animal
        idx = 0 # Initialize temporary indexing variable
        while idx < self.animals_count:
            # ASV should not navigate towards crocodile
            if self.animals[idx].action == "circumvent":
                idx += 1
                continue
            # ASV should not navigate towards an animal (platypus/turtle) that has already been circled
            if self.animals[idx].done:
                idx += 1
                continue
            # Compute distance to the animal
            dist = euclidean_distance(self.asv_pose, self.animals[idx].pose)
            # Get index of the nearest safe animal (platypus/turtle)
            if dist < min_dist:
                min_dist = dist
                index = idx
            idx += 1
        return index

    def safe_circling_radius(self, animal_index):
        '''
        Update actual radius of a particular animal so that the ASV stays outside
        the ideal radius of crocodile and critical radius of platypus/turtle.

        :param animal_index: Index of a particular animal, whose actual radius is to be updated

        :return: None
        '''
        assert animal_index < len(self.animals) # Sanity check
        for index, animal in enumerate(self.animals):
            if index == animal_index:
                continue
            rho, alpha, _ = cartesian_to_polar(self.asv_pose, self.animals[animal_index].pose) # Convert cartesian coordinates to polar coordinates
            rho_circumvent, alpha_circumvent, _ = cartesian_to_polar(self.asv_pose, self.animals[index].pose) # Convert cartesian coordinates to polar coordinates
            delta_alpha = abs(alpha - alpha_circumvent) # Required change in alpha
            dist_circumvent = math.sqrt(rho ** 2 + rho_circumvent ** 2 - 2 * rho * rho_circumvent * math.cos(delta_alpha))
            if animal.action == "circumvent": # For crocodile
                # Try to stay outside ideal radius of crocodile and critical radius of platypus/turtle
                r = dist_circumvent - self.animals[index].radius_ideal
                if r > self.animals[animal_index].radius_ideal:
                    r = self.animals[animal_index].radius_ideal
                if r < self.animals[animal_index].radius_critical:
                    r = self.animals[animal_index].radius_critical
                self.animals[animal_index].radius_actual = r # Update actual radius of the animal
            else: # For platupus and turtle
                # Try to stay outside critical radius of the other animals (platypus/turtle)
                r = dist_circumvent - self.animals[animal_index].radius_ideal
                if r > self.animals[index].radius_critical:
                    self.animals[index].radius_actual = self.animals[index].radius_critical
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_ideal
                elif dist_circumvent - self.animals[animal_index].radius_critical < self.animals[index].radius_critical:
                    print("Animals to be circled are too close to each other!")
                    print()
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_critical
                else:
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_critical
        if self.debug:
            print("Actual radius of {} is {:.4} m".format(self.animals[animal_index].name, float(self.animals[animal_index].radius_actual)))
            print()

    def choose_waypoint(self, rho, alpha, radius, clockwise=True, animal_index=None):
        '''
        Choose waypoint based on distance from a particular animal. For platupus and turtle, head
        towards the tangential point of bounding circle if far from animal, else start circling it.
        For crocodile, circumvent it by moving along the tangent to bounding circle.

        :param rho      : Distance to the animal (bounding circle center) w.r.t. WAM-V frame in meters
        :param alpha    : Direction of the animal (bounding circle center) w.r.t. WAM-V frame in radians
        :param radius   : Radius of the bounding circle around the animal in meters
        :param clockwise: Boolean flag indicating whether the circling direction is clockwise

        :return rho_new, alpha_new, beta_new: Polar coordinates of the tangential waypoint
        '''
        # Sanity check
        if animal_index is None:
            return 0, 0, 0
        # Minimum and maximum permissible radius for circling an animal
        r_min = radius + self.min_radius
        r_max = self.max_radius
        # Initialize polar coordinates of the new waypoint
        rho_new = rho
        alpha_new = alpha
        beta_new = 0
        # If ASV is far away from the animal
        if rho > 9 and self.animals[animal_index].asv_approaching:
            # For platypus and turtle
            if self.animals[animal_index].action != "circumvent":
                print("Approaching {}...".format(self.animals[animal_index].name))
                print()
                if self.debug:
                    print("rho: {:.4f} m, r_min: {:.4f} m, r_max: {:.4f} m".format(rho, r_min, r_max))
                rho_new = math.sqrt(rho ** 2 - r_min ** 2)
                r_min = r_min if r_min < rho else rho
                if clockwise:
                    alpha_new = alpha + math.asin(r_min / rho)
                else:
                    alpha_new = alpha - math.asin(r_min / rho)
            # For crocodile
            else:
                print("Circumventing {}...".format(self.animals[animal_index].name))
                print()
                if self.debug:
                    print("rho: {:.4f} m, r_min: {:.4f} m, r_max: {:.4f} m".format(rho, r_min, r_max))
                # If already away from the crocodile
                if r_min < rho:
                    if clockwise:
                        alpha_new = alpha + math.asin(r_min / rho)
                    else:
                        alpha_new = alpha - math.asin(r_min / rho)
                # If getting close to the crocodile
                else:
                    print("Getting close to the crocodile, correcting course...")
                    print()
                    r_min = rho - 1
                    if clockwise:
                        alpha_new = alpha + math.asin(r_min / rho)
                    else:
                        alpha_new = alpha - math.asin(r_min / rho)
        # If ASV is close to the animal (for platypus and turtle)
        else:
            direction = "clockwise" if self.animals[animal_index].action == "circle_cw" else "counter-clockwise"
            print("Circling {} in {} direction...".format(self.animals[animal_index].name, direction))
            print()
            # If ASV has just reached near the animal
            if self.animals[animal_index].asv_approaching:
                self.circle_tracker.reset(self.asv_pose.orientation.z) # Reset circle tracker to start accumulating yaw measurements
                self.animals[animal_index].asv_approaching = False # Reset the `asv_approaching` flag
            # If ASV drives out of range while circling the animal
            if rho > r_max:
                self.animals[animal_index].asv_circling = False # Reset the `asv_circling` flag
                self.animals[animal_index].asv_approaching = True # Set the `asv_approaching` flag (this also resets the circle tracker according to above logic)
                print("Drove out of range while circling {}, attempting again...".format(self.animals[animal_index].name))
                print()
            # If ASV is within range while circling the animal
            else:
                self.animals[animal_index].asv_circling = True # Set the `asv_circling` flag
                self.circle_tracker.update(self.asv_pose.orientation.z) # Start accumulating yaw measurements
            rho_new = math.sqrt(rho ** 2 + radius ** 2)
            if clockwise:
                beta_new = -math.atan2(radius, rho)
            else:
                beta_new = math.atan2(radius, rho)
            alpha_new = alpha - beta_new
        alpha_new = normalize_angle(alpha_new) # Normalize alpha within [-pi, pi)
        return rho_new, alpha_new, beta_new

    def near_animal(self, animal_index, dist_thresh):
        '''
        Check whether the ASV is near a particular animal.

        :param animal_index: Index of a particular animal, whose distance from ASV is to be measured
        :param dist_thresh : Threshold value to determine whether the ASV is near a particular animal

        :return result: Boolean flag indicating whether the ASV is near a particular animal
        '''
        dist = euclidean_distance(self.asv_pose, self.animals[animal_index].pose)
        result = dist < dist_thresh
        return result

    def circumvent_crocodile(self, orig_alpha, clockwise):
        '''
        Compute heading vector to safely circumvent the crocodile.

        :param orig_alpha: Heading of the original waypoint w.r.t. WAM-V frame in radians
        :param clockwise : Boolean flag indicating whether the circling direction is clockwise

        :return safe_alpha: Heading vector to safely circumvent the crocodile
        '''
        safe_alpha = orig_alpha # Initialize safe alpha
        circumvent_indices = [animal.index for animal in self.animals if animal.action == "circumvent"] # Indices of all crocodiles
        if len(circumvent_indices) == 0: # If there are no crocodiles
            if self.debug:
                print("There are no crocodiles in the scene!")
                print()
            return safe_alpha # No need to adjust alpha
        if len(circumvent_indices) >= 3: # If all (three) animals are crocodiles
            if self.debug:
                print("All the animals in the scene are crocodiles!")
                print()
        assert len(circumvent_indices) < 3 # Confirm that all (three) animals are not crocodiles
        multi_crocs_near_each_other = False # Boolean flag to check whether multiple (two) crocodiles are present near each other
        # If multiple (two) crocodiles are present, and their bounding circles are overlapping each other
        if len(circumvent_indices) == 2 and euclidean_distance(self.animals[circumvent_indices[0]].pose, self.animals[circumvent_indices[1]].pose) < self.animals[circumvent_indices[0]].radius_ideal + self.animals[circumvent_indices[1]].radius_ideal:
            multi_crocs_near_each_other = True # Set the `multi_crocs_near_each_other` flag
        alpha_range_dict = {} # Initialize dictionary to store alpha ranges
        nearest_rho_index = 0 # Initialize index of nearest rho
        nearest_rho = 1000000 # Initialize value of nearest rho
        for index in circumvent_indices:
            rho, alpha, _ = cartesian_to_polar(self.asv_pose, self.animals[index].pose) # Convert cartesian coordinates to polar coordinates
            if rho < nearest_rho:
                nearest_rho = rho
                nearest_rho_index = index
            r_ideal = self.animals[index].radius_ideal # Fetch ideal radius of animal
            _, alpha_cw, _ = self.choose_waypoint(rho, alpha, r_ideal, clockwise=True, animal_index=index) # Choose clockwise circling waypoint based on distance from animal
            _, alpha_ccw, _ = self.choose_waypoint(rho, alpha, r_ideal, clockwise=False, animal_index=index) # Choose counter-clockwise circling waypoint based on distance from animal
            # If both new alphas are equal, use any one (in this case alpha_ccw)
            if alpha_ccw == alpha_cw:
                safe_alpha = alpha_ccw
                return safe_alpha
            # Arrange the new alphas in ascending order
            alpha_min = alpha_cw if alpha_cw < alpha_ccw else alpha_ccw
            alpha_max = alpha_cw if alpha_cw > alpha_ccw else alpha_ccw
            # Store alpha range in dictionary
            alpha_range_dict[index] = (alpha_min, alpha_max)
        # If `alpha_range_dict` was never updated (sanity check)
        if len(alpha_range_dict) == 0:
            return safe_alpha
        if self.debug:
            print("Alpha Range Dictionary:")
            print(alpha_range_dict)
            print()
        # If multiple (two) crocodiles are present, and their bounding circles are overlapping each other, the ASV cannot traverse between them.
        # Update `alpha_range_dict`
        if multi_crocs_near_each_other:
            alpha_range_dict = [(min(alpha_range_dict[circumvent_indices[0]][0], alpha_range_dict[circumvent_indices[1]][0]), max(alpha_range_dict[circumvent_indices[0]][1], alpha_range_dict[circumvent_indices[1]][1]))]
        # Path optimization logic:
        #       - If alpha does not cross any bounding circle, it will stay unchanged
        #       - If alpha crosses one circle, choose the nearest tangential alpha of that bounding circle
        #       - If alpha crosses two circles, choose the nearest tangential alpha of the nearest bounding circle
        # In any case, choose the nearest alpha from the alpha range
        count = 0
        for key, value in alpha_range_dict.items():
            if angle_within_half_plane_range(safe_alpha, value[0], value[1]):
                if self.debug:
                    print("Current alpha {:.4} is within {:.4} and {:.4}".format(safe_alpha, value[0], value[1]))
                    print()
                count += 1
                if count == 2:
                    safe_alpha = alpha_range_dict[nearest_rho_index][1] if clockwise else alpha_range_dict[nearest_rho_index][0]
                    break
                else:
                    safe_alpha = value[1] if clockwise else value[0]
                safe_alpha += numpy.deg2rad(10) if clockwise else -numpy.deg2rad(10) # Further add synthetic repulsion factor to repel the ASV further away from the crocodile (added safety just-in-case)
                safe_alpha = normalize_angle(safe_alpha) # Normalize alpha within [-pi, pi)
        return safe_alpha

    def circle_completed(self, circle_tracker, animal_index):
        '''
        Check whether the ASV has finished circling a particular animal for complete 360°.

        :param circle_tracker: CircleTracker object
        :param animal_index  :  Index of animal being circled currently

        :return circle_completed: Boolean flag indicating whether the ASV has finished circling a particular animal
        '''
        if self.animals[animal_index].action == "circle_cw": # For platypus
            return circle_tracker.acc_yaw < -2.0 * math.pi
        elif self.animals[animal_index].action == "circle_ccw": # For turtle
            return circle_tracker.acc_yaw > 2.0 * math.pi
        else: # Circling progress for rocodile need not be checked
            if self.debug:
                print("Animal index {} corresponds to a crocodile, whose circling progress need not be checked!".format(self.animals[animal_index]))
                print()
            return False

    def task_completed(self):
        '''
        Check whether the ASV has finished circling all the safe animals (platypus/turtle) for complete 360°.

        :param: None

        :return result: Boolean flag indicating whether the ASV has finished circling all the safe animals (platypus/turtle)
        '''
        if self.animals_count == 0: # Sanity check during initialization of the task
            return False
        result = all([animal.done for animal in self.animals])
        return result

    def semantic_navigation(self):
        # Broadcast transform of `wamv/base_link` frame w.r.t. `world` frame
        self.tf_broadcaster.sendTransform((self.cur_position[0], self.cur_position[1], self.cur_position[2]), (self.cur_rotation.x, self.cur_rotation.y, self.cur_rotation.z, self.cur_rotation.w), rospy.Time.now(), 'wamv/base_link', 'world')
        # Initialize goal pose in polar coordinates
        rho_new, alpha_new, beta_new = 0, 0, 0
        # Make sure ASV's pose is updated before proceeding
        if self.asv_pose == Pose():
            return rho_new, alpha_new, beta_new
        # Make sure animals are detected before proceeding
        if self.animals_count == 0:
            return rho_new, alpha_new, beta_new
        # Report detected animals
        if self.init_debug:
            print()
            print("Detected {} animals:".format(self.animals_count))
            for i in range(self.animals_count):
                print("{}. {}".format(i+1, self.animals[i].name.capitalize()))
            print()
            self.init_debug = False # Make sure that initial debug messages are printed only once
        # Choose the nearest animal to circle
        next_index = self.next_animal_index()
        # Sanity checks
        if next_index is None:
            rospy.logwarn("No animal to circumvent!")
            all_crocodiles = all([animal.action == "circumvent" for animal in self.animals])
            assert not all_crocodiles
            return rho_new, alpha_new, beta_new
        assert self.animals[next_index].action != "circumvent"
        # Update safe circling radius of the target animal
        self.safe_circling_radius(next_index)
        # Get distance (rho) and direction (alpha) towards the target animal (in polar coordinates)
        rho, alpha, _ = cartesian_to_polar(self.asv_pose, self.animals[next_index].pose)
        # Get polar coordinates of the tangential waypoint of the bounding circle of the target animal
        rho_new, alpha_new, beta_new = self.choose_waypoint(rho, alpha, self.animals[next_index].radius_actual, clockwise = self.animals[next_index].action=="circle_cw", animal_index=next_index)
        if self.debug:
            print("Desired Heading: {:.4}".format(alpha_new))
        # Add synthetic repulsion factor to the desired heading in order to circumvent any crocodiles
        if not self.animals[next_index].asv_circling and not self.near_animal(next_index, dist_thresh=self.thresh_dist): # If ASV is not near any animal or circling any animal
            alpha_new = self.circumvent_crocodile(alpha_new, clockwise=self.animals[next_index].action == "circle_cw")
            if self.debug:
                print("Circumvention Heading: {:.4}".format(alpha_new))
        # Track circling progress
        if self.circle_completed(self.circle_tracker, next_index): # If target animal has been circled for complete 360°
            print("Circling Progress: ASV has finished circling the {}".format(self.animals[next_index].name))
            print()
            self.animals[next_index].done = True # Set the animal's `done` flag
            self.circle_tracker.reset(0) # Reset the circle tracker
        else: # If ASV is still circling the target animal
            print("Circling Progress: ASV has circled the {} for {:.4}°".format(self.animals[next_index].name, abs(numpy.rad2deg(self.circle_tracker.acc_yaw))))
            print()
        if self.debug:
            print("Next Local Polar Waypoint: {:.4}, {:.4}, {:.4}".format(rho_new, numpy.rad2deg(alpha_new), numpy.rad2deg(beta_new)))
            print()
        # Generate and publish `cmd_vel` message
        self.cmd_vel_msg = self.polar_controller.control(rho_new, alpha_new, beta_new) # Polar controller output
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        # Generate and publish `animal_model_viz` and `animal_label_viz` messages
        for j in range(self.animals_count):
            self.animal_model_viz_msg.header.frame_id = 'world'
            self.animal_label_viz_msg.header.frame_id = 'world'
            self.animal_model_viz_msg.header.stamp = rospy.Time.now()
            self.animal_label_viz_msg.header.stamp = rospy.Time.now()
            self.animal_model_viz_msg.ns = 'animals'
            self.animal_label_viz_msg.ns = 'animals'
            self.animal_model_viz_msg.action = 0 # 0 corresponds to add/modify object
            self.animal_label_viz_msg.action = 0 # 0 corresponds to add/modify object
            self.animal_model_viz_msg.type = 10 # 10 corresponds to mesh resource type
            self.animal_label_viz_msg.type = 9 # 9 corresponds to text view facing type
            if self.animals[j].name=='crocodile':
                self.animal_model_viz_msg.id = 0
                self.animal_label_viz_msg.id = 0
                self.animal_model_viz_msg.mesh_resource = 'package://vrx_gazebo/models/crocodile_buoy/meshes/crocodile.dae'
                self.animal_label_viz_msg.text = 'crocodile'
            elif self.animals[j].name=='platypus':
                self.animal_model_viz_msg.id = 1
                self.animal_label_viz_msg.id = 1
                self.animal_model_viz_msg.mesh_resource = 'package://vrx_gazebo/models/platypus_buoy/meshes/platypus.dae'
                self.animal_label_viz_msg.text = 'platypus'
            elif self.animals[j].name=='turtle':
                self.animal_model_viz_msg.id = 2
                self.animal_label_viz_msg.id = 2
                self.animal_model_viz_msg.mesh_resource = 'package://vrx_gazebo/models/turtle_buoy/meshes/turtle.dae'
                self.animal_label_viz_msg.text = 'turtle'
            self.animal_model_viz_msg.mesh_use_embedded_materials = True
            self.animal_model_viz_msg.pose = self.animals[j].pose
            self.animal_label_viz_msg.pose.position.x = self.animals[j].pose.position.x
            self.animal_label_viz_msg.pose.position.y = self.animals[j].pose.position.y
            self.animal_label_viz_msg.pose.position.z = self.animals[j].pose.position.z - 0.4
            self.animal_label_viz_msg.pose.orientation = self.animals[j].pose.orientation
            self.animal_model_viz_msg.scale.x = 1.0
            self.animal_model_viz_msg.scale.y = 1.0
            self.animal_model_viz_msg.scale.z = 1.0
            self.animal_label_viz_msg.scale.z = 0.4
            self.animal_label_viz_msg.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            self.animal_model_viz_pub.publish(self.animal_model_viz_msg)
            self.animal_label_viz_pub.publish(self.animal_label_viz_msg)

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.polar_controller = PolarController(config['k_rho'], config['k_alpha'], config['k_beta'], config['max_lin_vel'], config['max_ang_vel']) # Polar controller
        self.gps_offset  = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.min_radius  = config['min_radius'] # Minimum radius for circling an animal
        self.max_radius  = config['max_radius'] # Maximum radius for circling an animal
        self.thresh_dist = config['thresh_dist'] # Threshold distance to consider ASV being near an animal
        self.debug       = config['debug'] # Flag to enable/disable debug messages
        self.config      = config
        return config

################################################################################

if __name__ == "__main__":
    rospy.init_node('singaboat_semantic_navigation', anonymous=True)

    # SemanticNavigation class instance
    semantic_navigation_node = SemanticNavigation()

    # Dynamic reconfigure server
    semantic_navigation_node.dyn_reconf_srv = Server(SemanticNavigationConfig, semantic_navigation_node.config_callback)

    # Transform broadcaster
    semantic_navigation_node.tf_broadcaster = tf.TransformBroadcaster()

    # Subscribers
    rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, semantic_navigation_node.gps_callback)
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, semantic_navigation_node.imu_callback)
    rospy.Subscriber('/vrx/wildlife/animals/poses', GeoPath, semantic_navigation_node.wildlife_callback)

    # Publishers
    semantic_navigation_node.animal_model_viz_pub = rospy.Publisher('/rviz/animal_models', Marker, queue_size=10)
    semantic_navigation_node.animal_label_viz_pub = rospy.Publisher('/rviz/animal_labels', Marker, queue_size=10)
    semantic_navigation_node.cmd_vel_pub          = rospy.Publisher('/wamv/cmd_vel', Twist, queue_size=10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/wamv/sensors/gps/gps/fix', NavSatFix)
    rospy.wait_for_message('/wamv/sensors/imu/imu/data', Imu)
    rospy.wait_for_message('/vrx/wildlife/animals/poses', GeoPath)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            semantic_navigation_node.semantic_navigation()
            if semantic_navigation_node.task_completed():
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
