#!/usr/bin/env python3

import math
import numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import open3d
import matplotlib.pyplot as plt
import scipy.stats as stats
from itertools import combinations, groupby
from copy import deepcopy
import rospy
from geometry_msgs.msg import Pose # !!! Pose.orientation NOT used as a quaternion !!!
from sensor_msgs.msg import Image
from vrx_gazebo.srv import ColorSequence
from singaboat_vrx.common_utilities import gps_to_enu, enu_to_gps, quaternion_to_euler, euler_to_quaternion, local_to_global_tf

################################################################################

class BuoyDetector():
    '''
    Detects (classifies and localizes) round and marker buoys of different colors
    by fusing camera amd LIDAR data.
    '''
    def __init__(self):
        # Initialize buoy detector
        self.time_slot          = 0 # Time slot to detect objects in the scene
        self.task_name          = None # Name of the current VRX task
        self.asv_pose           = Pose() # Current ASV pose in global frame
        self.cv_bridge          = CvBridge() # CvBridge object
        self.rgb_objects        = [] # List of RGB objects detected by camera
        self.pc_centroids       = [] # List of centroids of point cloud clusters detected by LIDAR
        self.detected_objects   = [] # List containing semantic labels and positions (in local frame) of detected objects
        self.detected_obstacles = [] # List containing positions (in local frame) of detected obstacles
        self.config             = {} # Buoy detector configuration
        # ROS infrastructure
        self.cam_viz_msg        = None
        self.cam_viz_pub        = None
        self.object_msg         = None
        self.object_pub         = None
        self.obstacle_msg       = None
        self.obstacle_pub       = None
        self.dyn_reconf_srv     = None

    def task_callback(self, msg):
        self.task_name = msg.name
        # Objects absent in 0-5 sec slot, present in 5-10 sec slot, then `time_slot` resets to 0
        self.time_slot = msg.elapsed_time.secs % 10

    def gps_callback(self, msg):
        if self.asv_pose.orientation.z is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        self.asv_pose.position.x, self.asv_pose.position.y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        self.asv_pose.position.x += self.gps_offset * math.cos(self.asv_pose.orientation.z)
        self.asv_pose.position.y += self.gps_offset * math.sin(self.asv_pose.orientation.z)

    def imu_callback(self, msg):
        self.asv_pose.orientation.z = quaternion_to_euler(msg.orientation)[2]

    def camera_callback(self, msg):
        if self.task_name == "perception":
            min_area = 100
            max_area = 30000
        else:
            min_area = 50
            max_area = 30000
        # Try converting ROS Image message to OpenCV image
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(error)
        #-----------------------------#
        # WEATHER DETECTION ALGORITHM
        #-----------------------------#
        # ROI for weather detection
        img_roi_weather_detection = cv_image[520:720, 900:1180]
        # Convert from RGB to HSV
        hsv_img_weather_detection = cv2.cvtColor(img_roi_weather_detection, cv2.COLOR_BGR2HSV)
        # Clolr thresholds for weather detection
        lower_gray = numpy.array([0, 0, 0])
        upper_gray = numpy.array([100, 255, 255])
        # Threshold gray color to get binary image
        mask_gray = cv2.inRange(hsv_img_weather_detection, lower_gray, upper_gray)
        # Filter out gray pixels
        res_gray = cv2.bitwise_and(img_roi_weather_detection, img_roi_weather_detection, mask=mask_gray)
        # Compute weather score by averaging gray pixel values
        weather_score = res_gray[..., 2].mean()
        if weather_score > 15:
            sunny = True
            overcast = False
            weather = "Sunny"
        else:
            overcast = True
            sunny = False
            weather = "Overcast"
        # Add weather text to the weather image
        txt = "Weather: " + weather
        txt_origin = (5, 15)
        txt_color = (0, 255, 0)
        img_roi_weather_detection = cv2.putText(res_gray.copy(), txt, txt_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.75, txt_color, 1, cv2.LINE_AA)
        # Display weather detection image
        if self.debug:
            cv2.imshow("Weather Detection", img_roi_weather_detection)
            cv2.waitKey(1)
        #----------------------------#
        # OBJECT DETECTION ALGORITHM
        #----------------------------#
        # ROI for object detection
        crop_img_object_detection = cv_image[140:535, 0:]
        # Convert from RGB to HSV
        hsv_img_object_detection = cv2.cvtColor(crop_img_object_detection, cv2.COLOR_BGR2HSV)
        # Color thresholds for black buoys
        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([0, 0, 22])
        # Color thresholds for red buoys
        lower_red = numpy.array([153, 0, 0])
        upper_red = numpy.array([232, 255, 255])
        # Color thresholds for orange buoys
        lower_orange = numpy.array([0, 206, 50])
        upper_orange = numpy.array([71, 255, 255])
        # Color thresholds for green buoys
        lower_green = numpy.array([55, 61, 0])
        upper_green = numpy.array([90, 255, 255])
        # Color thresholds for white buoys
        lower_white = numpy.array([0, 0, 67])
        if sunny is True:
            upper_white = numpy.array([51, 0, 255]) # Higher threshold for sunny weather
            if self.debug:
                print("Weather appears to be sunny.")
                print()
        elif overcast is True:
            upper_white = numpy.array([51, 0, 151]) # Lower threshold for overcast weather
            if self.debug:
                print("Weather appears to be overcast.")
                print()
        # Perform color thresholding to get a binary image for each color
        mask_black = cv2.inRange(hsv_img_object_detection, lower_black, upper_black)
        mask_red = cv2.inRange(hsv_img_object_detection, lower_red, upper_red)
        mask_orange = cv2.inRange(hsv_img_object_detection, lower_orange, upper_orange)
        mask_green = cv2.inRange(hsv_img_object_detection, lower_green, upper_green)
        mask_white = cv2.inRange(hsv_img_object_detection, lower_white, upper_white)
        # Calculate centroid of the blob for each color
        m_black = cv2.moments(mask_black, False)
        m_red = cv2.moments(mask_red, False)
        m_orange = cv2.moments(mask_orange, False)
        m_green = cv2.moments(mask_green, False)
        m_white = cv2.moments(mask_white, False)
        # Get blob area for each color
        try:
            blob_area_black = m_black['m00']
        except ZeroDivisionError:
            blob_area_black = 0
        try:
            blob_area_red = m_red['m00']
        except ZeroDivisionError:
            blob_area_red = 0
        try:
            blob_area_orange = m_orange['m00']
        except ZeroDivisionError:
            blob_area_orange = 0
        try:
            blob_area_green = m_green['m00']
        except ZeroDivisionError:
            blob_area_green = 0
        try:
            blob_area_white = m_white['m00']
        except ZeroDivisionError:
            blob_area_white = 0
        if self.debug:
            print("Black Blob Area :{}".format(blob_area_black))
            print("Red Blob Area   :{}".format(blob_area_red))
            print("Orange Blob Area:{}".format(blob_area_orange))
            print("Green Blob Area :{}".format(blob_area_green))
            print("White Blob Area :{}".format(blob_area_white))
            print()
        min_blob_threshold = 60000
        rgb_objects_tmp = []
        # Detect black buoys
        if blob_area_black >= min_blob_threshold:
            blur_black = cv2.GaussianBlur(mask_black, (3, 3), 0) # Reduce noise by applying gaussian filter
            thresh_black = cv2.threshold(blur_black, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
            erode_black = cv2.erode(thresh_black, None, iterations=3) # Reduce false detections by applying erosion filter
            # Find contours in the pre-processed image, then select the largest one
            contours = cv2.findContours(erode_black.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            if self.debug:
                print("Number of Black Contours: {}".format(len(contours)))
            if len(contours) != 0:
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i]) # Area of the contour
                    if self.debug:
                        print("Black Contour {} Area: {}".format(i, area))
                    # Draw the contour if contour area is larger than a threshold
                    if area > min_area:
                        cv2.drawContours(crop_img_object_detection, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        # Algorithm to differentiate between marker buoy and round buoy
                        # Setup SimpleBlobDetector parameters
                        params = cv2.SimpleBlobDetector_Params()
                        # Filter by circularity
                        params.filterByCircularity = True
                        params.minCircularity = 0.2
                        params.maxCircularity = 0.9
                        # Create a detector with the parameters
                        ver = (cv2.__version__).split('.')
                        if int(ver[0]) < 3:
                            detector = cv2.SimpleBlobDetector(params)
                        else:
                            detector = cv2.SimpleBlobDetector_create(params)
                        # Detect circular blobs
                        blank_image = numpy.zeros((crop_img_object_detection.shape[0], crop_img_object_detection.shape[1], 3), numpy.uint8) # Create a blank image
                        cv2.drawContours(blank_image, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        gray_black = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY) # Convert to grayscale
                        keypoints = detector.detect(gray_black) # Detect circular blobs
                        # Calculate aspect ratio of the blobs
                        perimeter = cv2.arcLength(contours[i], True) # Perimeter of the contour
                        approx = cv2.approxPolyDP(contours[i], 0.05 * perimeter, True) # Approximation of shape of the contour
                        (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
                        aspect_ratio = float(w) / float(h) # Calculate aspect ratio of the bounding box
                        # Differentiate between marker buoy and round buoy based on circularity and aspect ratio
                        if len(keypoints) == 0 and aspect_ratio < 1: # Marker buoy is non-circular and has lower aspect ratio
                            # Compute center of the contour
                            M = cv2.moments(contours[i])
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            rgb_objects_tmp.append("mb_marker_buoy_black") # Append semantic label
                            rgb_objects_tmp.append([cX, cY]) # Append object position (in image coordinates)
                            cv2.putText(crop_img_object_detection, "Black Marker Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
                        else: # Round buoy is circular and has higher aspect ratio
                            # Compute center of the contour
                            M = cv2.moments(contours[i])
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            rgb_objects_tmp.append("mb_round_buoy_black") # Append semantic label
                            rgb_objects_tmp.append([cX, cY]) # Append object position (in image coordinates)
                            cv2.putText(crop_img_object_detection, "Black Round Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
            if self.debug:
                print()
        else:
            if self.debug:
                print("Number of Black Contours: 0")
                print()
        # Detect red buoys
        if blob_area_red >= min_blob_threshold:
            blur_red = cv2.GaussianBlur(mask_red, (3, 3), 0) # Reduce noise by applying gaussian filter
            thresh_red = cv2.threshold(blur_red, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
            erode_red = cv2.erode(thresh_red, None, iterations=3) # Reduce false detections by applying erosion filter
            # Find contours in the pre-processed image, then select the largest one
            contours = cv2.findContours(erode_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            if self.debug:
                print("Number of Red Contours: {}".format(len(contours)))
            if len(contours) != 0:
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i]) # Area of the contour
                    if self.debug:
                        print("Red Contour {} Area: {}".format(i, area))
                    # Calculate aspect ratio of the blobs
                    perimeter = cv2.arcLength(contours[i], True) # Perimeter of the contour
                    approx = cv2.approxPolyDP(contours[i], 0.05 * perimeter, True) # Approximation of shape of the contour
                    (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
                    aspect_ratio = float(w) / float(h) # Calculate aspect ratio of the bounding box
                    # Draw the contour if contour area is larger than a threshold and aspect ratio is appropriate
                    if area > min_area and aspect_ratio < 1: # Marker buoy has lower aspect ratio
                        cv2.drawContours(crop_img_object_detection, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        # Compute center of the contour
                        M = cv2.moments(contours[i])
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        rgb_objects_tmp.append("mb_marker_buoy_red") # Append semantic label
                        rgb_objects_tmp.append([cX,cY]) # Append object position (in image coordinates)
                        cv2.putText(crop_img_object_detection, "Red Marker Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
            if self.debug:
                print()
        else:
            if self.debug:
                print("Number of Red Contours: 0")
                print()
        # Detect orange buoys
        if blob_area_orange >= min_blob_threshold:
            blur_orange = cv2.GaussianBlur(mask_orange, (3, 3), 0) # Reduce noise by applying gaussian filter
            thresh_orange = cv2.threshold(blur_orange, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
            erode_orange = cv2.erode(thresh_orange, None, iterations=3) # Reduce false detections by applying erosion filter
            # Find contours in the pre-processed image, then select the largest one
            contours = cv2.findContours(erode_orange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            if self.debug:
                print("Number of Orange Contours: {}".format(len(contours)))
            if len(contours) != 0:
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i]) # Area of the contour
                    if self.debug:
                        print("Orange Contour {} Area: {}".format(i, area))
                    # Calculate aspect ratio of the blobs
                    perimeter = cv2.arcLength(contours[i], True) # Perimeter of the contour
                    approx = cv2.approxPolyDP(contours[i], 0.05 * perimeter, True) # Approximation of shape of the contour
                    (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
                    aspect_ratio = float(w) / float(h) # Calculate aspect ratio of the bounding box
                    # Draw the contour if contour area is larger than a threshold and aspect ratio is appropriate
                    if area > min_area and aspect_ratio >= 1: # Round buoy has higher aspect ratio
                        cv2.drawContours(crop_img_object_detection, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        # Compute center of the contour
                        M = cv2.moments(contours[i])
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        rgb_objects_tmp.append("mb_round_buoy_orange") # Append semantic label
                        rgb_objects_tmp.append([cX, cY]) # Append object position (in image coordinates)
                        cv2.putText(crop_img_object_detection, "Orange Round Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
            if self.debug:
                print()
        else:
            if self.debug:
                print("Number of Orange Contours: 0")
                print()
        # Detect green buoys
        if blob_area_green >= min_blob_threshold/1.5:
            blur_green = cv2.GaussianBlur(mask_green, (3, 3), 0) # Reduce noise by applying gaussian filter
            thresh_green = cv2.threshold(blur_green, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
            erode_green = cv2.erode(thresh_green, None, iterations=2) # Reduce false detections by applying erosion filter
            # Find contours in the pre-processed image, then select the largest one
            contours = cv2.findContours(erode_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            if self.debug:
                print("Number of Green Contours: {}".format(len(contours)))
            if len(contours) != 0:
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i]) # Area of the contour
                    if self.debug:
                        print("Green Contour {} Area: {}".format(i, area))
                    # Calculate aspect ratio of the blobs
                    perimeter = cv2.arcLength(contours[i], True) # Perimeter of the contour
                    approx = cv2.approxPolyDP(contours[i], 0.05 * perimeter, True) # Approximation of shape of the contour
                    (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
                    aspect_ratio = float(w) / float(h) # Calculate aspect ratio of the bounding box
                    # Draw the contour if contour area is larger than a threshold and aspect ratio is appropriate
                    if area > min_area and aspect_ratio < 1: # Marker buoy has lower aspect ratio
                        cv2.drawContours(crop_img_object_detection, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        # Compute center of the contour
                        M = cv2.moments(contours[i])
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        rgb_objects_tmp.append("mb_marker_buoy_green") # Append semantic label
                        rgb_objects_tmp.append([cX, cY]) # Append object position (in image coordinates)
                        cv2.putText(crop_img_object_detection, "Green Marker Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
            if self.debug:
                print()
        else:
            if self.debug:
                print("Number of Green Contours: 0")
                print()
        # Detect white buoys
        if blob_area_white >= min_blob_threshold:
            blur_white = cv2.GaussianBlur(mask_white, (3, 3), 0) # Reduce noise by applying gaussian filter
            thresh_white = cv2.threshold(blur_white, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
            erode_white = cv2.erode(thresh_white, None, iterations=1) # Reduce false detections by applying erosion filter
            # Find contours in the pre-processed image, then select the largest one
            contours = cv2.findContours(erode_white.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            if self.debug:
                print("Number of White Contours: {}".format(len(contours)))
            if len(contours) != 0:
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i]) # Area of the contour
                    if self.debug:
                        print("White Contour {} Area: {}".format(i, area))
                    # Calculate aspect ratio of the blobs
                    perimeter = cv2.arcLength(contours[i], True) # Perimeter of the contour
                    approx = cv2.approxPolyDP(contours[i], 0.05 * perimeter, True) # Approximation of shape of the contour
                    (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
                    aspect_ratio = float(w) / float(h) # Calculate aspect ratio of the bounding box
                    # Draw the contour if contour area is larger than a threshold and aspect ratio is appropriate
                    if area > min_area and aspect_ratio < 1: # Marker buoy has lower aspect ratio
                        cv2.drawContours(crop_img_object_detection, contours, i, (255, 255, 0), 5) # Draw the contour in cyan color
                        # Compute center of the contour
                        M = cv2.moments(contours[i])
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        if cY > 20: # Disregard false detections due to sky
                            rgb_objects_tmp.append("mb_marker_buoy_white") # Append semantic label
                            rgb_objects_tmp.append([cX, cY]) # Append object position (in image coordinates)
                            cv2.putText(crop_img_object_detection, "White Marker Buoy", (cX+25, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA) # Add object label in cyan color
            if self.debug:
                print()
        else:
            if self.debug:
                print("Number of White Contours: 0")
                print()
        # Display object detection image
        if self.debug:
            cv2.imshow("Object Detection", crop_img_object_detection)
            cv2.waitKey(1)
        # Generate and publish `cam_viz` message
        try: # Try converting OpenCV image to ROS Image message
            self.cam_viz_msg = self.cv_bridge.cv2_to_imgmsg(crop_img_object_detection, encoding="bgr8")
        except CvBridgeError as error:
            print(error)
        self.cam_viz_pub.publish(self.cam_viz_msg)
        # Update `rgb_objects` list
        rgb_objects = rgb_objects_tmp
        if self.task_name == "perception":
            # Update `self.rgb_objects` only if there are more detections than earlier
            if len(rgb_objects) > len(self.rgb_objects):
                self.rgb_objects = rgb_objects
            # Do not update `self.rgb_objects` if objects are absent (objects are absent in 0-5 sec slot)
            if self.time_slot <= 4:
                self.rgb_objects = []
        else:
            self.rgb_objects = rgb_objects # Update `self.rgb_objects` continuously (independent of `time_slot`)
        rgb_objects_tmp = [] # Clear the temporary list after each iteration
        if self.debug:
            print("RGB Objects:")
            print(self.rgb_objects)
            print()

    def lidar_callback(self, msg):
        if self.task_name == "perception":
            dbscan_min_points = 4
        else:
            dbscan_min_points = 3
        ros_point_cloud = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))) # Get point cloud data
        open3d_point_cloud = open3d.geometry.PointCloud() # Initialize Open3D point cloud with restriced ROI (for perception task)
        open3d_point_cloud_wide = open3d.geometry.PointCloud() # Initialize Open3D point cloud with wide ROI (for gymkhana task)
        if len(ros_point_cloud) != 0:
            xyz_filter = []
            xyz_filter_wide = []
            xyz = [(x, y, 0) for x, y, z in ros_point_cloud] # Get x, y; disregard z
            for i in range(len(xyz)):
                # Alter region of interest (ROI) based on field of view (FOV)
                if 1 < xyz[i][0] < 35 and abs(xyz[i][1]) < 20: # If 1 < x < 35 and -20 < y < 20
                    xyz_filter_wide.append(xyz[i])
                    if abs(xyz[i][1]/xyz[i][0]) < 1: # Only include points lying closer in X than in Y
                        xyz_filter.append(xyz[i])
                        if xyz[i][0] < 10: # If x < 10
                            if abs(xyz[i][1]/xyz[i][0]) < 0.8: # Only include points lying 1.25 (i.e. 1/0.8) times closer in X than in Y
                                xyz_filter.append(xyz[i])
            # If LIDAR point cloud is available, process it uing the DBSCAN data clustering algorithm
            if len(xyz_filter_wide) != 0 and len(xyz_filter) != 0:
                xyz_array = numpy.array(xyz_filter)
                open3d_point_cloud.points = open3d.utility.Vector3dVector(xyz_array)
                xyz_array_wide = numpy.array(xyz_filter_wide)
                open3d_point_cloud_wide.points = open3d.utility.Vector3dVector(xyz_array_wide)
                # Detect clusters present within the point cloud
                pc_clusters = numpy.array(open3d_point_cloud.cluster_dbscan(eps=0.3, min_points=dbscan_min_points, print_progress=False))
                pc_clusters_wide = numpy.array(open3d_point_cloud_wide.cluster_dbscan(eps=0.3, min_points=dbscan_min_points, print_progress=False))
                max_pc_cluster = pc_clusters.max()
                max_pc_cluster_wide = pc_clusters_wide.max()
                if self.debug:
                    print("Number of Point Cloud Clusters: {}".format(max_pc_cluster + 1))
                    print()
                # Calculate and rank centroids of point cloud clusters in `pc_clusters`
                pc_centroids_sum = numpy.zeros((max_pc_cluster + 1, 3)) # Used to store sum of all pc_centroids
                counters = numpy.zeros(max_pc_cluster + 1) # Used to store count of all pc_centroids
                pc_centroids = numpy.zeros((max_pc_cluster + 1, 3)) # Used to store updated pc_centroids
                for i in range(len(pc_clusters + 1)):
                    if pc_clusters[i] >= 0:
                        pc_centroids_sum[pc_clusters[i]] += xyz_array[i] # Update sum of pc_centroids
                        counters[pc_clusters[i]] += 1 # Update count of pc_centroids
                for i in range(max_pc_cluster + 1):
                    pc_centroids[i] = pc_centroids_sum[i] / counters[i] # Calculate centroids by averaging and update pc_centroids
                pc_centroids_ranked = self.rank_pc_centroids(pc_centroids) # Rank point cloud cluster centroids based on their local Y position
                # Calculate and rank centroids of point cloud clusters in `pc_clusters_wide`
                pc_centroids_sum_wide = numpy.zeros((max_pc_cluster_wide + 1, 3)) # Used to store sum of all pc_centroids_wide
                counters_wide = numpy.zeros(max_pc_cluster_wide + 1) # Used to store count of all pc_centroids_wide
                pc_centroids_wide = numpy.zeros((max_pc_cluster_wide + 1, 3)) # Used to store updated pc_centroids_wide
                for i in range(len(pc_clusters_wide + 1)):
                    if pc_clusters_wide[i] >= 0:
                        pc_centroids_sum_wide[pc_clusters_wide[i]] += xyz_array_wide[i] # Update sum of pc_centroids_wide
                        counters_wide[pc_clusters_wide[i]] += 1 # Update count of pc_centroids_wide
                for i in range(max_pc_cluster_wide + 1):
                    pc_centroids_wide[i] = pc_centroids_sum_wide[i] / counters_wide[i] # Calculate centroids by averaging and update pc_centroids_wide
                pc_centroids_ranked_wide = self.rank_pc_centroids(pc_centroids_wide) # Rank point cloud cluster centroids based on their local Y position
                if self.task_name == "perception":
                    # Update `self.pc_centroids` only if there are more detections than earlier
                    if len(pc_centroids_ranked) > len(self.pc_centroids):
                        self.pc_centroids = pc_centroids_ranked
                    # Do not update `self.pc_centroids` if objects are absent or have just appeared (objects appear at 5th sec of time_slot)
                    if self.time_slot < 5.5:
                        self.pc_centroids = []
                    # Once the objects are about to disappear, update `self.pc_centroids` and fuse camera-LIDAR data (objects disappear at 10th sec of time_slot)
                    if  8.9 < self.time_slot  < 9.8:
                        if len(self.rgb_objects) != 0 and len(self.pc_centroids) != 0:
                            rgb_objects_ranked = self.rank_rgb_objects(self.rgb_objects)
                            self.detected_objects = self.fuse_camera_lidar_data(self.pc_centroids, rgb_objects_ranked)
                else:
                    self.pc_centroids = pc_centroids_ranked # Update `self.pc_centroids` continuously (independent of `time_slot`)
                    # Update `self.detected_objects` for channel navigation
                    if len(self.rgb_objects) != 0 and len(self.pc_centroids) != 0:
                        rgb_objects_ranked = self.rank_rgb_objects(self.rgb_objects)
                        self.detected_objects = self.fuse_camera_lidar_data(self.pc_centroids, rgb_objects_ranked)
                    # Update `self.detected_obstacles` for obstacle avoidance
                    self.detected_obstacles = []
                    if len(pc_centroids_wide) != 0:
                        for i in range(len(pc_centroids_wide)):
                            self.detected_obstacles.append((pc_centroids_wide[i, 0:2]).tolist())
                    # Generate and publish `obstacle` message
                    self.obstacle_msg.data = sum(self.detected_obstacles, [])
                    self.obstacle_pub.publish(self.obstacle_msg)
                if self.debug:
                    print("Point Cloud Cluster Centroids:")
                    print(self.pc_centroids)
                    print()

    def rank_rgb_objects(self, rgb_objects):
        '''
        Rank RGB objects based on their local Y position (i.e. image X coordinate).

        :param rgb_objects: List of RGB objects (labels and positions in image coordinates)

        :return rgb_objects_ranked: List of RGB objects ranked by local Y position (i.e. image X coordinate)
        '''
        rgb_objects_pos = []
        for i in range(len(rgb_objects)):
            if (i + 1) % 2 == 0: # Append only object positions (not labels)
                rgb_objects_pos.append(rgb_objects[i])
        x = stats.rankdata(numpy.array(rgb_objects_pos)[:, 0], method='dense') # Rank from left to right (in image X coordinate)
        x_list = x.tolist()
        rgb_objects_ranked = []
        for i in range(int(len(rgb_objects) / 2)):
            index = x_list.index(i + 1) # Get index of (i + 1) rank from `x_list`
            rgb_objects_ranked.append(rgb_objects[2 * index]) # Append label of object ranked (i + 1) from `rgb_objects`
            rgb_objects_ranked.append(rgb_objects[2 * index + 1]) # Append position of object ranked (i + 1) from `rgb_objects`
        if self.debug:
            print("Ranked RGB Objects:")
            print(rgb_objects_ranked)
            print()
        return rgb_objects_ranked

    def rank_pc_centroids(self, pc_centroids):
        '''
        Rank point cloud cluster centroids based on their local Y position.

        :param pc_centroids: nx3 array of point cloud cluster centroids

        :return pc_centroids_ranked: nx3 array of point cloud cluster centroids ranked by local Y position
        '''
        slope = pc_centroids[:, 1] / pc_centroids[:, 0] # Calculate slope (m = y/x)
        y = stats.rankdata(slope, method='dense') # Rank from small to big (far to near), and right to left (in local Y coordinate)
        y = len(y) + 1 - y.astype(int) # Invert ranking (from left to right)
        y_list = y.tolist()
        pc_centroids_ranked = []
        for i in range(len(pc_centroids)):
            pc_centroids_ranked.append(pc_centroids[y_list.index(i + 1)]) # Append point cloud centroid ranked (i + 1) from `pc_centroids`
        pc_centroids_ranked = numpy.array(pc_centroids_ranked)
        if self.debug:
            print("Ranked Point Cloud Cluster Centroids:")
            print(pc_centroids_ranked)
            print()
        return pc_centroids_ranked

    def fuse_camera_lidar_data(self, pc_centroids_ranked, rgb_objects_ranked):
        '''
        Match RGB objects and point cloud cluster centroids based on their position.

        :param pc_centroids_ranked: Point cloud cluster centroids ranked by position (local Y position)
        :param rgb_objects_ranked : RGB objects ranked by position (local Y position)

        :return detected_objects: List of detected objects with semantic labels and positions (in local frame)
        '''
        detected_objects = []
        if len(rgb_objects_ranked) / 2 == len(pc_centroids_ranked):
            if self.debug:
                print("Camera and LIDAR detections are consistent.")
                print()
            # If camera and LIDAR detections are consistent, directly perform data fusion
            for i in range(len(pc_centroids_ranked)):
                detected_objects.append(rgb_objects_ranked[2 * i]) # Append object labels (derived from camera)
                detected_objects.append(pc_centroids_ranked[i, 0:2]) # Append object positions (derived from LIDAR)
                if self.task_name == "perception":
                    print("Publishing detected objects...")
                    print()
                    self.report_detected_objects(pc_centroids_ranked[i, 0], pc_centroids_ranked[i, 1], rgb_objects_ranked[2 * i]) # Publish detected objects
        elif len(rgb_objects_ranked) / 2 < len(pc_centroids_ranked) and self.task_name == "perception":
            if self.debug:
                print("Camera and LIDAR detections are inconsistent!")
                print("Camera has less number of detections than LIDAR!")
                print("Resolving inconsistency...")
                print()
            # If LIDAR has more number of detections than camera, relax point cloud clustering algoithm to remove false detections
            open3d_point_cloud = open3d.geometry.PointCloud()
            open3d_point_cloud.points = open3d.utility.Vector3dVector(pc_centroids_ranked)
            # Relax processing parameters required to detect a cluster: `eps` (maximum inter-point distance) and `min_points` (minimum number of points)
            pc_clusters = numpy.array(open3d_point_cloud.cluster_dbscan(eps=2, min_points=1, print_progress=False))
            max_pc_cluster = pc_clusters.max()
            if self.debug:
                print("Number of Point Cloud Clusters: {}".format(max_pc_cluster + 1))
                print()
            pc_centroids_sum = numpy.zeros((max_pc_cluster + 1, 3)) # Used to store sum of all pc_centroids
            counters = numpy.zeros(max_pc_cluster + 1) # Used to store count of all pc_centroids
            pc_centroids = numpy.zeros((max_pc_cluster + 1, 3)) # Used to store updated pc_centroids
            for i in range(len(pc_clusters + 1)):
                if pc_clusters[i] >= 0:
                    pc_centroids_sum[pc_clusters[i]] += pc_centroids_ranked[i] # Update sum of pc_centroids
                    counters[pc_clusters[i]] += 1 # Update count of pc_centroids
            for i in range(max_pc_cluster + 1):
                pc_centroids[i] = pc_centroids_sum[i] / counters[i] # Calculate centroids by averaging and update pc_centroids
            # Now, perform data fusion
            k = 0
            for i in range(len(counters)):
                for j in range(int(counters[i])):
                    detected_objects.append(rgb_objects_ranked[2 * i]) # Append object labels (derived from camera)
                    detected_objects.append(pc_centroids_ranked[k, 0:2]) # Append object positions (derived from LIDAR)
                    if self.task_name == "perception":
                        print("Publishing detected objects...")
                        print()
                        self.report_detected_objects(pc_centroids_ranked[k, 0], pc_centroids_ranked[k, 1], rgb_objects_ranked[2 * i]) # Publish detected objects
                    k += 1
        elif len(rgb_objects_ranked) / 2 > len(pc_centroids_ranked) and self.task_name == "perception":
            if self.debug:
                print("Camera and LIDAR detections are inconsistent!")
                print("Camera has more number of detections than LIDAR!")
                print("Resolving inconsistency...")
                print()
            # If camera has more number of detections than LIDAR, remove false detections due to far-away objects (e.g. trees & mountains)
            lidar_detections_num = len(pc_centroids_ranked) # Number of LIDAR detections (assumed to be true detections)
            rgb_objects_ranked = rgb_objects_ranked[:2*len(pc_centroids_ranked)] # Discard last few camera detections since most false detections are in upper-right corner
            # Now, perform data fusion
            for i in range(len(pc_centroids_ranked)):
                detected_objects.append(rgb_objects_ranked[2 * i]) # Append object labels (derived from camera)
                detected_objects.append(pc_centroids_ranked[i, 0:2]) # Append object positions (derived from LIDAR)
                if self.task_name == "perception":
                    print("Publishing detected objects...")
                    print()
                    self.report_detected_objects(pc_centroids_ranked[i, 0], pc_centroids_ranked[i, 1], rgb_objects_ranked[2 * i]) # Publish detected objects
        if self.debug:
            print("Detected Objects:")
            print(detected_objects)
            print()
        return detected_objects

    def report_detected_objects(self, pc_centroid_x, pc_centroid_y, semantic_label, orientation=0):
        '''
        Transform object positions to global frame, convert to GPS coordinates and
        publish GeoPoseStamped message along with `frame_id` being the `semantic_label`.

        :param pc_centroid_x : Local X position(s) of point cloud cluster centroid(s)
        :param pc_centroid_y : Local Y position(s) of point cloud cluster centroid(s)
        :param semantic_label: Semantic label(s) of RGB object(s)
        :param orientation   : Global orientation of object(s)/landmark(s) [this is useful for channel navigation, where channel gate orientations are computed for planning paths through them]

        :return: None
        '''
        obj_x, obj_y, _ = local_to_global_tf(self.asv_pose.position.x, self.asv_pose.position.y, self.asv_pose.orientation.z, pc_centroid_x + self.lidar_offset, pc_centroid_y, 0) # WAM-V frame is +0.7 m offset in local x-axis w.r.t. LIDAR frame
        obj_lat, obj_lon, _ = enu_to_gps(obj_x, obj_y)
        # Generate and publish `object` message
        self.object_msg.header.stamp = rospy.Time.now()
        self.object_msg.header.frame_id = semantic_label
        self.object_msg.pose.position.latitude = obj_lat
        self.object_msg.pose.position.longitude = obj_lon
        q = euler_to_quaternion(0, 0, orientation)
        self.object_msg.pose.orientation.x = q[0]
        self.object_msg.pose.orientation.y = q[1]
        self.object_msg.pose.orientation.z = q[2]
        self.object_msg.pose.orientation.w = q[3]
        self.object_pub.publish(self.object_msg)
        print("Object Name: {}".format(semantic_label))
        print("Object Location: {}° N, {}° E".format(obj_lat, obj_lon))
        print()

    def config_callback(self, config, level):
        # Handle updated configuration values
        self.gps_offset   = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.lidar_offset = config['lidar_offset'] # LIDAR offset w.r.t. WAM-V along X-axis
        self.debug        = config['debug'] # Flag to enable/disable debug messages
        self.config       = config
        return config

################################################################################

class GateDetector():
    '''
    Detects (classifies and localizes) channel gates constituted of different
    marker buoys by making use of `BuoyDetector`.
    '''
    def __init__(self):
        # Initialize gate detector
        self.buoy_detector    = BuoyDetector() # BuoyDetector class instance
        self.asv_pose         = Pose() # Current ASV pose in global frame
        self.detected_objects = [] # List of currently detected objects
        self.channel_buoys    = ["mb_marker_buoy_red", "mb_marker_buoy_white", "mb_marker_buoy_green", "mb_marker_buoy_black"] # Reference list of channel defining buoys
        # Red buoy variables
        self.red_buoy_pos      = [] # Position (x, y) of red buoy w.r.t. WAM-V frame
        self.red_buoy_detected = False # Boolean flag to check whether a red buoy has been detected
        # White buoy variables
        self.white_buoy_pos      = [] # Position (x, y) of white buoy w.r.t. WAM-V frame
        self.white_buoy_enu      = [] # Position (x, y) of white buoy w.r.t. global frame
        self.white_buoy_detected = False # Boolean flag to check whether a white buoy has been detected
        # Green buoy variables
        self.green_buoy_pos      = [] # Position (x, y) of currently detected green buoy w.r.t. WAM-V frame
        self.green_buoy_enu      = [] # Position (x, y) of currently detected green buoy w.r.t. global frame
        self.green_buoy_1_pos    = [] # Position (x, y) of green buoy 1 w.r.t. WAM-V frame
        self.green_buoy_1_enu    = [] # Position (x, y) of green buoy 1 w.r.t. global frame
        self.green_buoy_2_pos    = [] # Position (x, y) of green buoy 2 w.r.t. WAM-V frame
        self.green_buoy_2_enu    = [] # Position (x, y) of green buoy 2 w.r.t. global frame
        self.green_buoy_3_pos    = [] # Position (x, y) of green buoy 3 w.r.t. WAM-V frame
        self.green_buoy_3_enu    = [] # Position (x, y) of green buoy 3 w.r.t. global frame
        self.green_buoy_detected = False # Boolean flag to check whether a green buoy has been detected
        # Black buoy variables
        self.black_buoy_pos      = [] # Position (x, y) of black buoy w.r.t. WAM-V frame
        self.black_buoy_enu      = [] # Position (x, y) of black buoy w.r.t. global frame
        self.black_buoy_detected = False # Boolean flag to check whether a black buoy has been detected
        # Entry gate variables
        self.entrance_position    = []
        self.entrance_orientation = None
        self.entrance_detected    = False
        self.entrance_reported    = False
        # Middle gate variables
        self.gate_position      = []
        self.gate_1_position    = []
        self.gate_1_orientation = None
        self.gate_1_detected    = False
        self.gate_1_reported    = False
        self.gate_2_position    = []
        self.gate_2_orientation = None
        self.gate_2_detected    = False
        self.gate_2_reported    = False
        self.gate_3_position    = []
        self.gate_3_orientation = None
        self.gate_3_detected    = False
        self.gate_3_reported    = False
        # Exit gate variables
        self.exit_position    = []
        self.exit_orientation = None
        self.exit_detected    = False
        self.exit_reported    = False
        # ROS infrastructure
        self.config         = {} # Gate detector configuration
        self.dyn_reconf_srv = None # Dynamic reconfigure server

    def gps_callback(self, msg):
        if self.asv_pose.orientation.z is None: # If no yaw data is available, GPS offset cannot be compensated
            return
        lat = msg.latitude
        lon = msg.longitude
        self.asv_pose.position.x, self.asv_pose.position.y, _ = gps_to_enu(lat, lon)
        # WAM-V frame is +0.85 m offset in local x-axis w.r.t. GPS frame
        self.asv_pose.position.x += self.gps_offset * math.cos(self.asv_pose.orientation.z)
        self.asv_pose.position.y += self.gps_offset * math.sin(self.asv_pose.orientation.z)

    def imu_callback(self, msg):
        self.asv_pose.orientation.z = quaternion_to_euler(msg.orientation)[2]

    def local_to_global_pos(self, local_pos):
        '''
        Convert position coordinates (x, y) from local frame to global frame.

        :param local_pos: Position coordinates (x, y) in local frame

        :return global_pos: Position coordinates in global frame
        '''
        x, y, _ = local_to_global_tf(self.asv_pose.position.x, self.asv_pose.position.y, self.asv_pose.orientation.z, local_pos[0] + self.lidar_offset, local_pos[1],0)  # WAM-V frame is +0.7 m offset in local x-axis w.r.t. LIDAR frame
        global_pos = [x, y]
        return global_pos

    def gate_orientation(self, left_buoy_pos, right_buoy_pos, asv_orientation):
        '''
        Compute gate orientation based on buoy locations and ASV heading.

        :param left_buoy_pos  : Position (x, y) of the left (white/green/black) marker buoy in meters w.r.t. WAM-V frame
        :param right_buoy_pos : Position (x, y) of the right (red) marker buoy in meters w.r.t. WAM-V frame
        :param asv_orientation: Orientation of the ASV in radians

        :return gate_orientation: Orientation of the detected gate in radians w.r.t. global frame
        '''
        normal_slope = -(left_buoy_pos[0]-right_buoy_pos[0])/(left_buoy_pos[1]-right_buoy_pos[1]) # Slope of line normal to the line joining centers of left and right buoys
        gate_orientation = math.atan(normal_slope) + asv_orientation
        return gate_orientation

    def detect_entrance(self):
        '''
        Detect channel entrance by detecting white and red buoy pair.

        :param: None

        :return: None
        '''
        self.white_buoy_detected = False # Initialize white buoy detection flag
        self.red_buoy_detected   = False # Initialize red buoy detection flag
        if len(self.detected_objects) > 0 and not self.entrance_detected:
            if self.debug:
                print("Detected Objects:")
                print(self.detected_objects)
                print()
            for i in range(int(len(self.detected_objects)/2)):
                # If detected object is a white buoy
                if self.detected_objects[2 * i] == self.channel_buoys[1]:
                    if self.debug:
                        print("White buoy detected!")
                        print()
                    self.white_buoy_detected = True # Set white buoy detection flag
                    self.white_buoy_pos = numpy.array(self.detected_objects[2 * i + 1]) # Extract position of detected object
                    self.white_buoy_enu = self.local_to_global_pos(numpy.array(self.white_buoy_pos )) # Convert position coordinates to global frame (ENU format)
                # If detected object is a red buoy
                elif self.detected_objects[2 * i] == self.channel_buoys[0]:
                    if self.debug:
                        print("Red buoy detected!")
                        print()
                    self.red_buoy_detected = True # Set red buoy detection flag
                    self.red_buoy_pos = numpy.array(self.detected_objects[2 * i + 1]) # Extract position of detected object
                # If both white and red buoys are detected
                if self.white_buoy_detected and self.red_buoy_detected:
                    gate_width = math.dist(self.white_buoy_pos, self.red_buoy_pos) # Compute distance between white and red buoy
                    # Check whether gate width is within acceptable limits
                    if self.min_gate_width < gate_width < self.max_gate_width:
                        if self.debug:
                            print("Entrance detected!")
                            print()
                        self.entrance_position = ((self.white_buoy_pos + self.red_buoy_pos) / 2).tolist() # Position of the entry gate w.r.t. WAM-V frame
                        self.entrance_orientation = self.gate_orientation(self.white_buoy_pos, self.red_buoy_pos, self.asv_pose.orientation.z) # Global orientation
                        self.entrance_detected = True # Set entrance detection flag
                        if self.debug:
                            print("Entrance:")
                            print(self.entrance_position)
                            print()
                    else:
                        if self.debug:
                            print("Gate measurements seem absurd!")
                            print("Gate Width: {}".format(gate_width))
                            print()
                        self.white_buoy_detected = False # Reset white buoy detection flag
                        self.red_buoy_detected   = False # Reset red buoy detection flag

    def detect_gate(self):
        '''
        Detect channel gates by detecting green and red buoy pairs.

        :param: None

        :return: None
        '''
        self.green_buoy_detected = False # Initialize green buoy detection flag
        self.red_buoy_detected   = False # Initialize red buoy detection flag
        if len(self.detected_objects) > 0 and self.entrance_detected:
            if self.debug:
                print("Detected Objects:")
                print(self.detected_objects)
                print()
            for i in range(int(len(self.detected_objects)/2)):
                # If detected object is a green buoy
                if self.detected_objects[2 * i] == self.channel_buoys[2]:
                    if self.debug:
                        print("Green buoy detected!")
                        print()
                    self.green_buoy_detected = True # Set green buoy detection flag
                    self.green_buoy_pos = self.detected_objects[2 * i + 1] # Extract position of detected object
                    self.green_buoy_enu = self.local_to_global_pos(numpy.array(self.green_buoy_pos )) # Convert position coordinates to global frame (ENU format)
                    dist_0 = math.dist(self.green_buoy_enu, self.white_buoy_enu) # Distance between currently detected green buoy and previously detected white buoy
                    if dist_0 < self.min_gate_width/2:
                        if self.debug:
                            print("Wrong green buoy detected!")
                            print()
                        self.green_buoy_detected = False # Reset green buoy detection flag
                    else:
                        if not self.green_buoy_1_pos == []: # If green buoy 1 has been detected already
                            dist_1 = math.dist(self.green_buoy_enu, self.green_buoy_1_enu) # Distance between currently detected green buoy and previously detected green buoy 1
                            if not self.green_buoy_2_pos == []: # If green buoy 2 has been detected already
                                if not self.green_buoy_3_pos == []: # If green buoy 3 has been detected already
                                    break # All green buoys have been detected
                                dist_2 = math.dist(self.green_buoy_enu, self.green_buoy_2_enu) # Distance between currently detected green buoy and previously detected green buoy 2
                                if dist_2 > self.min_gate_width and dist_1 > self.min_gate_width:
                                    if self.debug:
                                        print("Green buoy 3 detected!")
                                        print()
                                else:
                                    if self.debug:
                                        print("Green buoy 2 detected again!")
                                        print()
                                    if self.gate_2_detected:
                                        self.green_buoy_detected = False
                                    continue
                            else:
                                if dist_1 > self.min_gate_width:
                                    if self.debug:
                                        print("Green buoy 2 detected!")
                                        print()
                                else:
                                    if self.debug:
                                        print("Green buoy 1 detected again!")
                                        print()
                                    if self.gate_1_detected:
                                        self.green_buoy_detected = False
                                    continue
                        else:
                            if self.debug:
                                print("Green buoy 1 detected!")
                                print()
                # If detected object is a red buoy
                elif self.detected_objects[2 * i] == self.channel_buoys[0]:
                    if self.debug:
                        print("Red buoy detected!")
                        print()
                    self.red_buoy_detected = True # Set red buoy detection flag
                    self.red_buoy_pos = numpy.array(self.detected_objects[2 * i + 1]) # Extract position of detected object
                # If both green and red buoys are detected
                if self.green_buoy_detected and self.red_buoy_detected:
                    if self.debug:
                        print("Gate detected!")
                        print()
                    gate_width = math.dist(self.green_buoy_pos, self.red_buoy_pos) # Compute distance between green and red buoy
                    self.gate_position = ((self.green_buoy_pos + self.red_buoy_pos) / 2).tolist() # Position of the gate w.r.t. WAM-V frame
                    gate_dist = math.dist(self.gate_position, [0,0]) # Compute distance between the detected gate and WAM-V frame
                    # Check whether gate width is within acceptable limits and distance to gate is also reasonable
                    if self.min_gate_width < gate_width < self.max_gate_width and gate_dist < self.dist_to_gate:
                        if self.gate_1_detected: # If gate 1 has been detected already, this is either gate 2 or gate 3
                            if self.gate_2_detected: # If gate 1 and gate 2 have been detected already, this is gate 3
                                self.green_buoy_3_pos = self.green_buoy_pos
                                self.green_buoy_3_enu =  self.green_buoy_enu
                                self.gate_3_position = self.gate_position # Local position (w.r.t. WAM-V frame)
                                self.gate_3_orientation = self.gate_orientation(self.green_buoy_3_pos, self.red_buoy_pos, self.asv_pose.orientation.z) # Global orientation
                                self.gate_3_detected = True # Set gate 3 detection flag
                                gate_id = 3
                            else: # If gate 1 has been detected already but gate 2 has not been detected yet, this is gate 2
                                self.green_buoy_2_pos = self.green_buoy_pos
                                self.green_buoy_2_enu = self.green_buoy_enu
                                self.gate_2_position = self.gate_position # Local position (w.r.t. WAM-V frame)
                                self.gate_2_orientation = self.gate_orientation(self.green_buoy_2_pos, self.red_buoy_pos, self.asv_pose.orientation.z) # Global orientation
                                self.gate_2_detected = True # Set gate 2 detection flag
                                gate_id = 2
                        else: # Else this is gate 1
                            self.green_buoy_1_pos = self.green_buoy_pos
                            self.green_buoy_1_enu = self.green_buoy_enu
                            self.gate_1_position = self.gate_position # Local position (w.r.t. WAM-V frame)
                            self.gate_1_orientation = self.gate_orientation(self.green_buoy_1_pos, self.red_buoy_pos, self.asv_pose.orientation.z) # Global orientation
                            self.gate_1_detected = True # Set gate 1 detection flag
                            gate_id = 1
                        if self.debug:
                            print("Gate {}:".format(gate_id))
                            print(self.gate_position)
                            print()
                    else:
                        if self.debug:
                            print("Gate measurements seem absurd!")
                            print("Gate Width: {}".format(gate_width))
                            print("Distance to Gate: {}".format(gate_dist))
                            print()
                        self.green_buoy_detected = False # Reset green buoy detection flag
                        self.red_buoy_detected   = False # Reset red buoy detection flag
                        self.gate_position = [] # Reset detected gate position

    def detect_exit(self):
        '''
        Detect channel exit by detecting black and red buoy pair.

        :param: None

        :return: None
        '''
        self.black_buoy_detected = False # Initialize black buoy detection flag
        self.red_buoy_detected   = False # Initialize red buoy detection flag
        if len(self.detected_objects) > 0 and self.entrance_detected:
            if self.debug:
                print("Detected Objects:")
                print(self.detected_objects)
                print()
            for i in range(int(len(self.detected_objects)/2)):
                # If detected object is a black buoy
                if self.detected_objects[2 * i] == self.channel_buoys[3]:
                    if self.debug:
                        print("Green buoy detected!")
                        print()
                    self.black_buoy_detected = True # Set black buoy detection flag
                    self.black_buoy_pos = numpy.array(self.detected_objects[2 * i + 1]) # Extract position of detected object
                    self.black_buoy_enu = self.local_to_global_pos(numpy.array(self.black_buoy_pos)) # Convert position coordinates to global frame (ENU format)
                    dist_0 = math.dist(self.black_buoy_enu, self.white_buoy_enu) # Distance between currently detected black buoy and previously detected white buoy
                    dist_1 = math.dist(self.black_buoy_enu, self.green_buoy_enu) # Distance between currently detected black buoy and previously detected green buoy (if any)
                    if dist_0 < self.min_gate_width/2 or dist_1 < self.min_gate_width/2:
                        if self.debug:
                            print("Wrong black buoy detected!")
                            print()
                        self.black_buoy_found = False # Reset black buoy detection flag
                # If detected object is a red buoy
                elif self.detected_objects[2 * i] == self.channel_buoys[0]:
                    if self.debug:
                        print("Red buoy detected!")
                        print()
                    self.red_buoy_detected = True # Set red buoy detection flag
                    self.red_buoy_pos = numpy.array(self.detected_objects[2 * i + 1])# Extract position of detected object
                # If both black and red buoys are detected
                if self.black_buoy_detected and self.red_buoy_detected:
                    if self.debug:
                        print("Exit detected!")
                        print()
                    gate_width = math.dist(self.black_buoy_pos, self.red_buoy_pos) # Compute distance between black and red buoy
                    # Check whether gate width is within acceptable limits
                    if self.min_gate_width < gate_width <  self.max_gate_width:
                        self.exit_position = ((self.black_buoy_pos + self.red_buoy_pos) / 2).tolist() # Position of the exit gate w.r.t. WAM-V frame
                        self.exit_orientation = self.gate_orientation(self.black_buoy_pos, self.red_buoy_pos, self.asv_pose.orientation.z) # Global orientation
                        self.exit_detected = True # Set entrance detection flag
                        if self.debug:
                            print("Exit:")
                            print(self.exit_position)
                            print()
                    else:
                        if self.debug:
                            print("Gate measurements seem absurd!")
                            print("Gate Width: {}".format(gate_width))
                            print()
                        self.black_buoy_detected = False # Reset black buoy detection flag
                        self.red_buoy_detected   = False # Reset red buoy detection flag

    def config_callback(self, config, level):
        # Handle updated configuration values
        # Buoy detector parameters
        self.buoy_detector.gps_offset   = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.buoy_detector.lidar_offset = config['lidar_offset'] # LIDAR offset w.r.t. WAM-V along X-axis
        self.buoy_detector.debug        = config['debug'] # Flag to enable/disable buoy detector debug messages
        # Gate detector parameters
        self.min_gate_width  = config['min_gate_width'] # Minimum width of channel gate
        self.max_gate_width  = config['max_gate_width'] # Maximum width of channel gate
        self.dist_to_gate    = config['dist_to_gate'] # Distance between the detected gate and WAM-V frame
        self.gps_offset      = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.lidar_offset    = config['lidar_offset'] # LIDAR offset w.r.t. WAM-V along X-axis
        self.debug           = config['debug'] # Flag to enable/disable gate detector debug messages
        self.config          = config
        return config

################################################################################

class ShapeDetector:
    '''
    Detects shapes of contours in an image by fitting an approximate polygon.
    ---------------
    Contour Shapes:
    ---------------
    Unknown              : If the shape has less than 3 vertices
    Triangle             : If the shape has 3 vertices
    Square               : If the shape has 4 vertices and approximately 1:1 aspect ratio
    Horizontal Rectangle : If the shape has 4 vertices and >1 aspect ratio
    Vertical Rectangle   : If the shape has 4 vertices and <1 aspect ratio
    Cross                : If the shape has 12 vertices
    Circle               : Otherwise
    '''
    def __init__(self):
        pass

    def detect(self, contour, eps=0.05, ar_tol=0.25):
        '''
        Detect approximated shape of a contour.

        :param contour: OpenCV `contour` object
        :param eps    : Maximum distance between the original curve and its approximation
        :param ar_tol : Tolerance of aspect ratio (width/height) to differentiate between square and rectangle

        :return shape: Approximated shape of a contour
        '''
        perimeter = cv2.arcLength(contour, True) # Perimeter of the contour
        approx = cv2.approxPolyDP(contour, eps * perimeter, True) # Approximation of shape of the contour
        # If the shape has less than 3 vertices, it is probably a line or a point
        if len(approx) <= 2:
            shape = "Unknown"
        # If the shape has 3 vertices, it is a triangle
        elif len(approx) == 3:
            shape = "Triangle"
        # If the shape has 4 vertices, it is either a square or a rectangle
        elif len(approx) == 4:
            (x, y, w, h) = cv2.boundingRect(approx) # Fit a bounding box to the approximated shape of the contour
            aspect_ratio = w / float(h) # Calculate aspect ratio of the bounding box
            # If the shape has aspect ratio that is approximately equal to 1, it is a square
            if (1-ar_tol) <= aspect_ratio <= (1+ar_tol):
                shape = "Square"
            # If the shape has aspect ratio that is more than 1, it is a horizontal rectangle
            elif aspect_ratio > (1+ar_tol):
                shape = "Horizontal Rectangle"
            # If the shape has aspect ratio that is less than 1, it is a vertical rectangle
            else:
                shape = "Vertical Rectangle"
        # If the shape has 12 vertices, it is a cross
        elif len(approx) == 12:
            shape = "Cross"
        # Otherwise, the shape is assumed to be a circle
        else:
            shape = "Circle"
        return shape

################################################################################

# Reference dictionaries storing HSV thresholding constants (1 to 3 means from brightest to darkest)
HSV_DICT_1 = {"Red":    {"lower": [0, 180, 146],  "upper": [0, 255, 255]},
              "Orange": {"lower": [0, 206, 50],   "upper": [71, 255, 255]},
              "Black":  {"lower": [0, 0, 0],      "upper": [0, 0, 22]},
              "Green":  {"lower": [55, 100, 101], "upper": [90, 255, 255]},
              "White":  {"lower": [0, 0, 67],     "upper": [51, 0, 151]},
              "Gray":   {"lower": [0, 0, 0],      "upper": [100, 255, 255]},
              "Blue":   {"lower": [103, 84, 222], "upper": [219, 255, 255]},
              "Yellow": {"lower": [6, 84, 208],   "upper": [30, 255, 255]}}
HSV_DICT_2 = {"Red":    {"lower": [0, 201, 0],    "upper": [19, 255, 255]},
              "Orange": {"lower": [0, 206, 50],   "upper": [71, 255, 255]},
              "Black":  {"lower": [0, 0, 0],      "upper": [0, 0, 22]},
              "Green":  {"lower": [55, 108, 93],  "upper": [98, 255, 161]},
              "White":  {"lower": [0, 0, 67],     "upper": [51, 0, 151]},
              "Gray":   {"lower": [0, 0, 0],      "upper": [100, 255, 255]},
              "Blue":   {"lower": [115, 84, 22],  "upper": [219, 255, 255]},
              "Yellow": {"lower": [6, 217, 119],  "upper": [46, 255, 139]}}
HSV_DICT_3 = {"Red":    {"lower": [0, 201, 0],    "upper": [19, 255, 255]},
              "Orange": {"lower": [0, 206, 50],   "upper": [71, 255, 255]},
              "Black":  {"lower": [0, 0, 0],      "upper": [0, 0, 22]},
              "Green":  {"lower": [55, 204, 29],  "upper": [99, 255, 146]},
              "White":  {"lower": [0, 0, 67],     "upper": [51, 0, 151]},
              "Gray":   {"lower": [0, 0, 0],      "upper": [100, 255, 255]},
              "Blue":   {"lower": [115, 39, 45],  "upper": [129, 255, 255]},
              "Yellow": {"lower": [6, 217, 16],   "upper": [46, 255, 150]}}
# Reference dictionary storing blob area thresholding constants
MIN_BLOB_AREA_DICT = {"Red":    8000,
                      "Orange": 15000,
                      "Black":  15000,
                      "Green":  15000,
                      "White":  15000,
                      "Gray":   15000,
                      "Blue":   15000,
                      "Yellow": 10000}
# Reference dictionary storing contour area thresholding constants
MIN_CONTOUR_AREA_DICT = {"Red":    10,
                         "Orange": 10,
                         "Black":  10,
                         "Green":  10,
                         "White":  10,
                         "Gray":   10,
                         "Blue":   10,
                         "Yellow": 3}

################################################################################

class ColoredRegionDetector:
    '''
    Detects colored regions within an image by making use of `ShapeDetector`.
    '''
    def __init__(self, colors, debug=False):
        self.shape_detector  = ShapeDetector() # ShapeDetector class instance
        self.cv_bridge       = CvBridge() # CvBridge object
        self.colors          = colors # List of interested colors to be filtered from image
        self.masks           = {} # Masks for each color
        self.contours        = {} # Contours of each color
        self.areas           = {} # Areas of contours of each color
        self.centroids       = {} # Centroids of blobs of each color
        self.shapes          = {} # Shapes of contours of each color
        self.image           = None # Current camera frame
        self.img_roi         = None # Cropped current camera frame based on region of interest (ROI)
        self.image_weather   = None # Image for weather detection
        self.img_roi_weather = None # Cropped image for weather detection based on region of interest (ROI)
        self.record_time     = 0 # Count record time
        self.brightness      = [0, 0, 0, 0, 0] # Record the brightness
        self.debug           = debug # Flag to enable/disable colored region detector debug messages
        # ROS infrastructure
        self.cam_viz_pub = rospy.Publisher('/rviz/cam_viz', Image, queue_size=10)

    def reset(self):
        '''
        Reset all detections.

        :param: None

        :return: None
        '''
        self.masks = {}
        self.contours = {}
        self.areas = {}
        self.centroids = {}
        self.shapes = {}

    def hsv_dict(self):
        '''
        Return appropriate HSV thresholding dictionary based on brightness of the scene.

        :param: None

        :return hsv_dict: Appropriate HSV thresholding dictionary
        '''
        if self.image_weather is None: # Sanity check
            return HSV_DICT_1
        img_weather = self.image_weather[520:720, 900:1180]
        weather_score = img_weather[..., 2].mean()
        record_cycle = 10
        finish_time = 50
        if self.record_time < finish_time:
            # Update record time before completing record
            self.record_time += 1
            if self.record_time % record_cycle == 0:
                # Record brightness of every instant (used for averaging)
                i = int(self.record_time / record_cycle) - 1
                self.brightness[i] = weather_score
            # Use real-time brightness before averaging data is ready
            avg_brightness = weather_score
        else:
            # Use average brightness once averaging data is ready
            avg_brightness = numpy.mean(self.brightness)
            # Display weather detection image
            if self.debug:
                # Add brightness text to the weather image
                txt_brightness_avg = "Average Brightness: " + str(round(avg_brightness, 2))
                txt_origin = (5, 15)
                txt_color = (0, 255, 0)
                self.img_roi_weather = cv2.putText(img_weather.copy(), txt_brightness_avg, txt_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.75, txt_color, 1, cv2.LINE_AA)
                cv2.imshow("Weather Detection", self.img_roi_weather)
        # Choose HSV thresholding dictionary based on brightness
        if avg_brightness > 55:
            hsv_dict = HSV_DICT_1
        elif avg_brightness > 45:
            hsv_dict = HSV_DICT_2
        else:
            hsv_dict = HSV_DICT_3
        return hsv_dict

    def detect(self, image=None, weather_image=None):
        '''
        Detect colored regions within an image.

        :param image        : Image for colored region detection
        :param weather_image: Reference image for weather detection

        :return colored_regions: Colored regions within the image
        '''
        self.reset()
        if image is None: # Sanity check
            return
        # Update images
        self.image = image
        self.image_weather = weather_image
        # Crop ROI
        self.img_roi = self.image[0:535, 0:]
        hsv_img = cv2.cvtColor(self.img_roi, cv2.COLOR_BGR2HSV)
        # Choose HSV thresholding dictionary based on brightness
        hsv_dict = self.hsv_dict()
        # Update mask and contours for each color
        for color in self.colors:
            if color in hsv_dict:
                self.masks[color] = cv2.inRange(hsv_img, numpy.array(hsv_dict[color]["lower"]), numpy.array(hsv_dict[color]["upper"]))
                area = cv2.moments(self.masks[color], False)['m00']
                if self.debug:
                    print("Color: {}\tArea: {:.2f}".format(color, area))
                    print()
                if area > MIN_BLOB_AREA_DICT[color]:
                    # Reduce noise
                    blur = cv2.GaussianBlur(self.masks[color], (3, 3), 0) # Reduce noise by applying gaussian filter
                    thresh = cv2.threshold(blur, 45, 255, cv2.THRESH_BINARY)[1] # Apply binary thresholding
                    erode = cv2.erode(thresh, None, iterations=2) # Reduce false detections by applying erosion filter
                    dilate = cv2.dilate(erode, None, iterations=1) # Reduce false detections by applying dilation filter
                    # Find contours in the pre-processed image, then select the largest one
                    contours = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours = contours[0] if len(contours) == 2 else contours[1]
                    # Process the contours
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if self.debug:
                            print("Contour Area: {}".format(area))
                            print()
                        if area > MIN_CONTOUR_AREA_DICT[color]:
                            # Update color to contours map and contour areas map
                            if color in self.contours:
                                self.contours[color].append(contour)
                                self.areas[color].append(area)
                            else:
                                self.contours[color] = [contour]
                                self.areas[color] = [area]
                            # Update color to shapes map
                            shape = self.shape_detector.detect(contour, eps=0.03, ar_tol=0.25)
                            if color in self.shapes:
                                self.shapes[color].append(shape)
                            else:
                                self.shapes[color] = [shape]
                            # Compute centroid of each contour
                            M = cv2.moments(contour)
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            if color in self.centroids:
                                self.centroids[color].append([cX, cY])
                            else:
                                self.centroids[color] = [[cX, cY]]
                contour_num = len(self.contours[color]) if color in self.contours else 0
                if self.debug:
                    print("Detected {} contours of {} color.".format(contour_num, color))
                    print()
            else:
                if self.debug:
                    print("Unrecognized Color: {}".format(color))
                    print()
        colored_regions = self.color_to_centroid(drop_shapes=["Horizontal Rectangle"]) # Drop detections pertaining to horizontal rectangles
        self.display_detections() # Display colored region detections (color contours labelled with shape text)
        return colored_regions

    def color_to_centroid(self, drop_shapes=None):
        '''
        Format detections by mapping detected colors to their respective centroids.

        :param drop_shapes: Shapes not to be considered while formatting

        :return output: Formatted color-to-centroid detections
        '''
        drop = ["Unknown"]
        if drop_shapes is not None:
            drop.extend(drop_shapes)
        assert len(self.contours) == len(self.shapes)
        if self.debug:
            print("Dropping shapes for color_to_centroid:")
            print(drop)
            print()
        output = []
        for color, centroids in self.centroids.items():
            assert color in self.shapes
            shapes = self.shapes[color]
            assert len(centroids) == len(shapes)
            for centroid, shape in zip(centroids, shapes):
                if shape not in drop:
                    output.append(color)
                    output.append(centroid)
        return output

    def color_to_shape(self, drop_shapes=None):
        '''
        Format detections by mapping detected colors to their respective shapes.

        :param drop_shapes: Shapes not to be considered while formatting

        :return output: Formatted color-to-shape detections
        '''
        drop = ["Unknown"]
        if drop_shapes is not None:
            drop.extend(drop_shapes)
        if self.debug:
            print("Dropping shapes for color_to_shape:")
            print(drop)
            print()
        output = {color: [] for color in self.shapes.keys()}
        for color, shapes in self.shapes.items():
            for shape in shapes:
                if shape not in drop:
                    output[color].append(shape)
        for color in output.copy():
            if len(output[color]) == 0:
                output.pop(color, None)
        return output

    def color_shape_to_centroid(self):
        '''
        Format detections by mapping detected colors and shapes to their respective centroids.

        :param: None

        :return output: Formatted color-shape-to-centroid detections
        '''
        assert len(self.centroids) == len(self.shapes)
        output = {}
        for color, shapes in self.shapes.items():
            assert color in self.centroids
            assert len(shapes) == len(self.centroids[color])
            for shape, centroid in zip(shapes, self.centroids[color]):
                output[(color, shape)] = centroid # Assumes that no two objects have the same color and shape
        return output

    def display_detections(self):
        '''
        Display detected colored regions within an image and label them semantically.

        :param: None

        :return: None
        '''
        if len(self.contours) != 0:
            contours = []
            contour_image = self.img_roi.copy()
            for color, contour in self.contours.items():
                contours.extend(contour)
                for i in range(len(contour)):
                    # Add detection color and shape text to the image
                    txt_detection_color = color
                    if str(self.shapes[color][i]) == "Vertical Rectangle":
                        txt_detection_shape = "Light Buoy Display"
                    elif str(self.shapes[color][i]) == "Horizontal Rectangle":
                        txt_detection_shape = "Rectangle"
                    else:
                        txt_detection_shape = str(self.shapes[color][i])
                    txt_color = (255, 255, 0) # Cyan color
                    cv2.putText(contour_image, txt_detection_color, (self.centroids[color][i][0], self.centroids[color][i][1]-35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, txt_color, 2, cv2.LINE_AA)
                    cv2.putText(contour_image, txt_detection_shape, (self.centroids[color][i][0], self.centroids[color][i][1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, txt_color, 2, cv2.LINE_AA)
            cv2.drawContours(contour_image, contours, -1, (255, 255, 0), 2) # Draw the contours in cyan color
            if self.debug:
                # Display colored region detection image
                cv2.imshow("Colored Region Detection", contour_image)
                cv2.waitKey(1)
            # Generate and publish `cam_viz` message
            try: # Try converting OpenCV image to ROS Image message
                cam_viz_msg = self.cv_bridge.cv2_to_imgmsg(contour_image, encoding="bgr8")
            except CvBridgeError as error:
                print(error)
            self.cam_viz_pub.publish(cam_viz_msg)

################################################################################

class ColorSequenceDetector:
    '''
    Detects and decodes light buoy color sequence by making use of `ColoredRegionDetector`.
    '''
    def __init__(self, colors, timeout=30, debug=False):
        self.colored_region_detector = ColoredRegionDetector(colors=colors, debug=debug) # ColoredRegionDetector class instance
        self.cv_bridge               = CvBridge() # CvBridge object
        self.image                   = None # Current camera frame
        self.color_sequence          = [] # List to store the color sequence
        self.black_index             = 0 # Index of black color in the color sequence (used to correct the order of color sequence)
        self.init_time               = rospy.get_time() # Initial timestamp (time when an object of this class is instantiated)
        self.last_color_time         = rospy.get_time() # Timestamp when last color was detected
        self.timeout                 = timeout # Color sequence detector timeout in seconds
        self.debug                   = debug # Flag to enable/disable color sequence detector debug messages

    def camera_callback(self, msg):
        # Try converting ROS Image message to OpenCV image
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(error)
            print()

    def detect(self):
        '''
        Detect and decode color sequence displayed by the light buoy.

        :param: None

        :return: None
        '''
        while self.image is None:
            rospy.sleep(rospy.Duration(secs=0, nsecs=1000*1000*500)) # Wait for camera data
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            image = self.image
            image_weather = self.image
            # Detect colored regions
            self.colored_region_detector.detect(image, image_weather)
            color_to_shape = self.colored_region_detector.color_to_shape(drop_shapes=["Horizontal Rectangle"])
            if len(color_to_shape) == 0:
                if self.debug:
                    print("No regular-shaped colored regions detected!")
                    print()
            if self.debug:
                print("Detected Colored Regions:")
                print(color_to_shape)
                print()
            # Update color sequence
            updated_for_this_frame = False
            for color, shapes in color_to_shape.items():
                for idx, shape in enumerate(shapes):
                    if shape == "Vertical Rectangle":
                        updated_for_this_frame = True
                        self.push_color(color)
                        break
                    else:
                        time_diff = rospy.get_time() - self.last_color_time # Use time difference to detect black color
                        if time_diff > 1.5:
                            self.black_index = len(self.color_sequence)
                            if self.debug:
                                print("Light buoy display not detected for too long, recording black index as {}".format(self.black_index))
                                print()
                if updated_for_this_frame:
                    break # Only update once for each frame
            if self.debug:
                print("Current Color Sequence: {}".format(self.color_sequence))
                print()
            # Check if color sequence is complete and correct
            if self.sequence_complete():
                if self.debug:
                    print("Complete Color Sequence: {}".format(self.color_sequence))
                    print()
                self.sort_sequence()
                if self.debug:
                    print("Sorted Color Sequence: {}".format(self.color_sequence))
                    print()
                print("Color Sequence: {}".format(self.color_sequence))
                print()
                if self.sequence_valid():
                    if self.debug:
                        print("Color sequence is valid!")
                        print()
                    if self.report_sequence():
                        if self.debug:
                            print("Color sequence is correct!")
                            print()
                        return self.decode_sequence()
                    else:
                        if self.debug:
                            print("Color sequence is incorrect, starting over...")
                            print()
                        self.color_sequence = [] # Start over
                else:
                    if self.debug:
                        print("Color sequence is invalid, starting over...")
                        print()
                    self.color_sequence = [] # Start over
            # Check timeout (either no signal or detection failed)
            if rospy.get_time() - self.init_time > self.timeout:
                print("Color sequence detector timeout, dock to a random bay!")
                print()
                return "Random", "Random"
            rate.sleep()

    def push_color(self, color):
        '''
        Push unique color into the color sequence. Start over if some color(s) is/are missed.

        :param color: New color to be pushed into the color sequence

        :return: None
        '''
        # If no color has been pushed so far, push the current color irrespective of any checks
        if len(self.color_sequence) == 0:
            self.color_sequence.append(color)
            self.last_color_time = rospy.get_time()
            return
        # Check if any color(s) is/are missed:
        # If color sequence is not yet complete and current color was already detected in the past (except for immediately previous detection), it means that some color(s) is/are missed!
        # Case 1: Start over from current color detection if any color(s) is/are missed
        # Case 2: Update color sequence if the current color was never detected before
        # Case 3: Do nothing if the current color is same as the last color
        color_skipped = False
        for i in range(len(self.color_sequence) - 1):
            if color == self.color_sequence[i]: # Case 1
                color_skipped = True
                break
        if color_skipped:
            if self.debug:
                print("Missed a color!")
                print()
            self.color_sequence = [color]
            self.black_index = 0
        elif color != self.color_sequence[-1]: # Case 2
            self.color_sequence.append(color)
        self.last_color_time = rospy.get_time()

    def sequence_complete(self):
        '''
        Check whether color sequence is detected completely (3 or more colors should be detected).

        :param: None

        :return result: Boolean flag determining whether the color sequence is detected completely
        '''
        result = len(self.color_sequence) >= 3
        return result

    def sort_sequence(self):
        '''
        Sort color sequence logically based on black index.

        :param: None

        :return: None
        '''
        assert len(self.color_sequence) == 3
        if self.debug:
            print("Black index is {}".format(self.black_index))
            print()
        new_sequence = []
        for i in range(3):
            new_sequence.append(self.color_sequence[(i + self.black_index) % 3])
        self.color_sequence = new_sequence

    def sequence_valid(self):
        '''
        Check whether the detected color sequence is valid (3 colors should be distinct).

        :param: None

        :return result: Boolean flag determining whether the detected color sequence is valid
        '''
        result = len(self.color_sequence) == 3 and self.color_sequence[0] != self.color_sequence[1] and self.color_sequence[1] != self.color_sequence[2] and self.color_sequence[2] != self.color_sequence[0]
        return result

    def report_sequence(self):
        '''
        Reports the detected color sequence to the VRX Scan-Dock-Deliver service.

        :param: None

        :return result: Boolean flag determining whether the reported color sequence was correct
        '''
        assert len(self.color_sequence) == 3
        rospy.wait_for_service("/vrx/scan_dock_deliver/color_sequence")
        try:
            srv_client = rospy.ServiceProxy("/vrx/scan_dock_deliver/color_sequence", ColorSequence)
            result = srv_client(self.color_sequence[0], self.color_sequence[1], self.color_sequence[2])
            return result
        except rospy.ServiceException as error:
            print("Service call failed: %s" % error)
            print()

    def decode_sequence(self):
        '''
        Decode the detected color sequence to determine the docking bay placard color and shape.

        :param: None

        :return color, shape: Color and shape of the target docking bay placard
        '''
        assert len(self.color_sequence) == 3
        color = self.color_sequence[0]
        shape_color_code = self.color_sequence[2]
        shape = "Unknown"
        if shape_color_code == "Red":
            shape = "Circle"
        elif shape_color_code == "Green":
            shape = "Triangle"
        elif shape_color_code == "Blue":
            shape = "Cross"
        elif shape_color_code == "Yellow":
            shape = "Horizontal Rectangle"
        else:
            if self.debug:
                print("Unknown Color Code: {}".format(shape_color_code))
                print()
        return color, shape

################################################################################

class DockDetector:
    '''
    Detects dock using point cloud data from LIDAR.
    '''
    def __init__(self, debug=False):
        self.o3d_cloud     = None # Open3D point cloud
        self.cloud         = None # Point cloud
        self.cropped_cloud = None # Point cloud within restriced ROI
        self.clusters      = [] # Point cloud clusters
        self.debug         = debug # Flag to enable/disable dock detector debug messages

    def lidar_callback(self, msg):
        cloud_data = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        cloud_data = [[x, y, z] for x, y, z in cloud_data]
        if self.o3d_cloud is None:
            self.o3d_cloud = open3d.geometry.PointCloud()
        self.o3d_cloud.points = open3d.utility.Vector3dVector(numpy.array(cloud_data))

    def reset(self):
        '''
        Reset all detections.

        :param: None

        :return: None
        '''
        self.cloud = None
        self.cropped_cloud = None
        self.clusters = []

    def point_cloud_valid(self):
        '''
        Check whether the received point cloud is valid (i.e. does it have points).

        :param: None

        :return result: Boolean flag indicating whether the received point cloud has points
        '''
        result = self.cloud.has_points() and self.cropped_cloud.has_points()
        return result

    def update_cloud(self, cloud):
        '''
        Update the point cloud.

        :param cloud: Point cloud

        :return result: Boolean flag determining validity of the point cloud
        '''
        if cloud is None:
            self.cloud = deepcopy(self.o3d_cloud)
            self.cropped_cloud = deepcopy(self.o3d_cloud)
        else:
            self.cloud = cloud
            self.cropped_cloud = cloud
        result = self.point_cloud_valid()
        return result

    def crop_cloud(self):
        '''
        Fit bounding box to the point cloud and crop it.

        :param: None

        :return: None
        '''
        min_bounds = numpy.array([1, -50, -10])
        max_bounds = numpy.array([100, 50, 10])
        box = open3d.geometry.AxisAlignedBoundingBox(min_bounds, max_bounds)
        if self.debug:
            print("Bounding Box: {}".format(box))
            print()
        self.cropped_cloud = self.cropped_cloud.crop(box)

    def cluster(self):
        '''
        Detect and group local point cloud clusters together.

        :param: None

        :return result: Boolean result of the clustering operation
        '''
        if not self.point_cloud_valid(): # Sanity check
            if self.debug:
                print("Received invalid point cloud, cannot perform clustering!")
                print()
            result = False
            return result
        self.clusters = numpy.array(self.cropped_cloud.cluster_dbscan(eps=1.5, min_points=3, print_progress=False))
        if len(self.clusters) > 0:
            max_cluster = self.clusters.max()
            if self.debug:
                print("Number of Point Cloud Clusters: {}".format(max_cluster + 1))
                print()
            # Assign different colors to different clusters
            colors = plt.get_cmap("tab20")(self.clusters / (max_cluster if max_cluster > 0 else 1))
            colors[self.clusters < 0] = 0  # Note: cluster == -1 denotes a noise point
            self.cropped_cloud.colors = open3d.utility.Vector3dVector(colors[:, :3])
            result = True
            return result
        result = False
        return result

    def filter_bays_by_collinear_centroids(self, cloud=None):
        '''
        Filter docking bays based on collinearity w.r.t. their centroids.

        :param cloud: Updated point cloud

        :return bays: List of bays (poses) satisfying the collinearity criterion
        '''
        # 0. Initialization
        bays = []
        self.reset()
        if not self.update_cloud(cloud):
            if self.debug:
                print("Updating point cloud failed!")
                print()
            return bays
        # 1. Crop point cloud
        self.crop_cloud()
        if not self.cluster():
            if self.debug:
                print("Clustering point cloud failed!")
                print()
            return bays
        # 2. Update bay centroids and remove redundant centroids
        centroids = self.cluster_bounding_box_centroids()
        collapsed_centroids = self.collapse_centroids(centroids)
        # 3. Test the collinearity criterion
        bay_ids = self.collinearity_test(collapsed_centroids)
        if self.debug:
            print("Bay IDs passing the collinearity test:")
            print(bay_ids)
            print()
        if bay_ids is not None:
            # Collinear bay poses share the same averaged normals
            theta = self.average_normal_angle(collapsed_centroids, bay_ids)
            for bay_id in bay_ids:
                x, y, _ = collapsed_centroids[bay_id]
                bays.append([x, y, theta])
            if self.debug:
                print("Bay poses w.r.t. WAM-V:")
                print(bays)
                print()
        return bays

    def filter_bays_by_plane_normals(self, cloud=None):
        '''
        Filter docking bays based on collinearity w.r.t. their plane normals (i.e. their heading).

        :param cloud: Updated point cloud

        :return bays: List of bays satisfying the coplanarity criterion
        '''
        # 0. Initialization
        self.reset()
        if not self.update_cloud(cloud):
            if self.debug:
                print("Updating point cloud failed!")
                print()
            return []
        # 1. Crop point cloud
        self.crop_cloud()
        # 2. Filter vertical planes from the point cloud
        self.cropped_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=30))
        normals = numpy.asarray(self.cropped_cloud.normals)
        points = numpy.asarray(self.cropped_cloud.points)
        filtered_points = []
        for idx, normal in enumerate(normals):
            if abs(normal[2]) < 0.2: # Only keep horizontal plane normals
                filtered_points.append(points[idx])
        if self.debug:
            print("Number of points before applying plane filter: {}".format(len(points)))
            print("Number of points after applying plane filter:  {}".format(len(filtered_points)))
            print()
        if len(filtered_points) == 0: # Check empty points
            return []
        self.cropped_cloud.points = open3d.utility.Vector3dVector(numpy.array(filtered_points))
        self.cropped_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=30))
        self.cropped_cloud.normalize_normals()
        # 3. Cluster points to form separate planes
        if not self.cluster():
            if self.debug:
                print("Clustering point cloud failed!")
                print()
            return []
        # 4. Update plane centroids and normals
        centroids = self.cluster_bounding_box_centroids()
        normals = self.cluster_normals()
        # 5. Filter bays based on centroids and normals
        bays = self.filter_bays_by_centroids_and_normals(centroids, normals)
        return bays

    def filter_bays_by_centroids_and_normals(self, centroids, normals):
        '''
        Filter docking bays based on orientation of their normals, and distance between their centroids.

        :param centroids: Point cloud cluster centroids
        :param normals  : Normals to the planes formed by the point cloud clusters

        :return bays: List of bays satisfying the orientation and distance criteria
        '''
        bays = []
        if centroids is None or normals is None: # Sanity check
            if self.debug:
                print("Cannot filter docking bays without any centroids or normals!")
                print()
            return bays
        if len(centroids) < 3:
            if self.debug:
                print("Only {} point cloud clusters were detected!".format(len(centroids)))
                print()
            return bays
        assert len(centroids) == len(normals)
        # Keep horizontal plane normals
        ids = []
        for idx, normal in enumerate(normals):
            if abs(normal[-1]) < 0.2:
                ids.append(idx)
        if self.debug:
            print("Horizontal Normal IDs:")
            print(ids)
            print()
        # Keep planes with nearly parallel normals that point in the same direction
        similar_ids = []
        thetas = {}
        # Get angle (theta) of all normals in horizontal plane
        for id in ids:
            nx, ny, nz = normals[id]
            theta = math.atan2(ny, nx)
            thetas[id] = theta
        if self.debug:
            print("Angle of all normals in horizontal plane:")
            print([(idx, theta / math.pi * 180) for idx, theta in thetas.items()])
            print()
        # Cluster planes by similar horizontal normals
        for id1, theta1 in thetas.items():
            parallel_ids = [id1]
            for id2, theta2 in thetas.items():
                if id1 == id2:
                    continue
                if abs(thetas[id2] - thetas[id1]) < 15 / 180 * math.pi:
                    parallel_ids.append(id2)
            similar_ids.append(parallel_ids)
        assert len(similar_ids) == len(ids)
        for item in similar_ids:
            item.sort()
        similar_ids = list(k for k, _ in groupby(similar_ids))
        # Sort to consider the largest group first
        similar_ids = sorted(similar_ids, key=lambda elem: len(elem), reverse=True)
        if self.debug:
            print("Similar Horizontal Normal IDs:")
            print(similar_ids)
            print()
        # Keep planes with appropriate inter-centroid distance
        bay_ids = []
        for sim_ids in similar_ids:
            if len(sim_ids) == 3:
                if self.debug:
                    print("Got 3 similar IDs!")
                    print()
                all_xy = [(centroids[i][0], centroids[i][1]) for i in sim_ids]
                is_bay = True
                for i in range(2):
                    x1, y1 = all_xy[i]
                    x2, y2 = all_xy[i + 1]
                    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                    if dist < 2 or dist > 7:
                        is_bay = False
                        break
                if is_bay:
                    if self.debug:
                        print("These 3 centroids meet bay interval requirements!")
                        print()
                    bay_ids = sim_ids
                    break
            # At least 2 docking bays should be detected
            if len(sim_ids) > 1:
                hori_dists = []
                for id in sim_ids:
                    # Try to merge clusters in the vertical plane
                    x, y, z = centroids[id]
                    dist = numpy.sqrt(x ** 2 + y ** 2)
                    far_enough = True
                    for d in hori_dists:
                        if abs(dist - d) < 1:
                            far_enough = False
                            break
                    if far_enough:
                        hori_dists.append(dist)
                        bay_ids.append(id)
            # Only consider the first similar group since bay pattern is unique in the scene
            if len(bay_ids) > 1:
                break
        if self.debug:
            print("Detected {} docking bays with the following IDs:".format(len(bay_ids)))
            print(bay_ids)
            print()
        # Organize docking bays with centroids and normals (in clockwise sense)
        for bay_id in bay_ids:
            x, y, z = centroids[bay_id]
            theta = thetas[bay_id]
            bays.append([x, y, theta])
        # Sort in decreasing order by theta in WAM-V frame
        # Note: Corner case is bays stand across negative Y-axis of WAM-V frame
        bays = sorted(bays, key=lambda elem: math.atan2(elem[1], elem[0]), reverse=True)
        if self.debug:
            print("Bay poses w.r.t. WAM-V:")
            print(bays)
            print()
        return bays

    def filter_plane(self, cloud=None):
        '''
        Filter single plane from point cloud (used when surrounding environment only contains one major plane)

        :param cloud: Updated point cloud

        :return plane_model, inlier_cloud: Plane model (i.e. [a, b, c, d] in ax + by + cz + d = 0), inliers of the point cloud
        '''
        self.reset()
        self.update_cloud(cloud)
        self.crop_cloud()
        if self.point_cloud_valid():
            [a, b, c, d], inliers = self.cropped_cloud.segment_plane(distance_threshold=0.05, ransac_n=5, num_iterations=1000)
            plane_model = [a, b, c, d]
            inlier_cloud = self.cropped_cloud.select_by_index(inliers)
            return plane_model, inlier_cloud
        plane_model = inlier_cloud = None
        return plane_model, inlier_cloud

    def cluster_centroids(self):
        '''
        Compute centroids of point cloud clusters by averaging point locations within each cluster.

        :param: None

        :return centroids: Centroids of point cloud clusters
        '''
        assert len(self.clusters) > 0
        max_cluster = self.clusters.max()
        points = numpy.asarray(self.cropped_cloud.points)
        points_sum = numpy.zeros((max_cluster + 1, 3))
        points_count = numpy.zeros((max_cluster + 1, 1))
        # Average point locations within each cluster to get cluster centroids
        for point, cluster in zip(points, self.clusters):
            # Ignore noise points
            if cluster < 0:
                continue
            points_count[cluster] += 1
            points_sum[cluster] += point
        assert all(points_count)
        centroids = points_sum / points_count
        if self.debug:
            print("Point Cloud Centroids:")
            print(centroids)
            print()
        return centroids

    def cluster_bounding_box_centroids(self):
        '''
        Compute centroids of boxes bounding the point cloud clusters.

        :param: None

        :return centroids: Centroids of boxes bounding the point cloud clusters
        '''
        assert len(self.clusters) > 0
        max_cluster = self.clusters.max()
        centroids = numpy.zeros((max_cluster + 1, 3))
        for i in range(max_cluster + 1):
            indices = numpy.where(self.clusters == i)[0].tolist()
            cloud = self.cropped_cloud.select_by_index(indices)
            centroid = cloud.get_center()
            centroids[i] = centroid
        if self.debug:
            print("Point Cloud Bounding Box Centroids:")
            print(centroids)
            print()
        return centroids

    def collapse_centroids(self, centroids, threshold=2):
        '''
        Remove redundant centroids that are close enough in horizontal plane.

        :param centroids: Point cloud cluster centroids
        :param threshold: Threshold for removing redundant centroids

        :return collapsed_centroids: Sorted, isolated centroids
        '''
        centroids_num = len(centroids)
        if centroids_num <= 1:
            if self.debug:
                print("Got less than 2 centroids, no need to collapse!")
                print()
            return centroids
        sorted_centroids = sorted(centroids, key=lambda elem: math.atan2(elem[1], elem[0]), reverse=True)
        collapsed_centroids = [sorted_centroids[0]]
        for i in range(1, centroids_num):
            cur_x, cur_y = sorted_centroids[i][0], sorted_centroids[i][1]
            last_x, last_y = collapsed_centroids[-1][0], collapsed_centroids[-1][1]
            dist = math.sqrt((cur_x - last_x) ** 2 + (cur_y - last_y) ** 2)
            if dist > threshold:
                collapsed_centroids.append(sorted_centroids[i])
        if self.debug:
            print("Number of centroids before collapsing: {}".format(centroids_num))
            print("Number of centroids after collapsing:  {}".format(len(collapsed_centroids)))
            print()
            print("Collapsed Centroids:")
            print(collapsed_centroids)
            print()
        return collapsed_centroids

    def cluster_normals(self):
        '''
        Compute cluster point normals.

        :param: None

        :return normals_avg: Average point normal within each cluster
        '''
        assert len(self.clusters) > 0
        self.cropped_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=2, max_nn=30))
        if not self.cropped_cloud.has_normals():
            if self.debug:
                print("Point cloud has no normals!")
                print()
            return None
        self.cropped_cloud.orient_normals_towards_camera_location()
        normals = numpy.asarray(self.cropped_cloud.normals)
        # Unify normal directions outwards w.r.t. LIDAR
        outward_normals = []
        for normal in normals:
            outward_normals.append(-normal)
        self.cropped_cloud.normals = open3d.utility.Vector3dVector(numpy.array(outward_normals))
        # Average point normals within each cluster to get point normals
        normals = numpy.asarray(self.cropped_cloud.normals)
        max_cluster = self.clusters.max()
        normals_avg = numpy.zeros((max_cluster + 1, 3))
        for normal, cluster in zip(normals, self.clusters):
            # Ignore noise points
            if cluster < 0:
                continue
            normals_avg[cluster] += normal
            normals_avg[cluster] = normals_avg[cluster] / numpy.linalg.norm(normals_avg[cluster])
        if self.debug:
            print("Point Cloud Cluster Normals:")
            print(normals_avg)
            print()
        return normals_avg

    def collinearity_test(self, centroids):
        '''
        Test the collinearity criterion for given centroids.

        :param centroids: Point cloud cluster centroids

        :return ids: IDs of 3 centroids that form a line (if any)
        '''
        centroids_num = len(centroids)
        if centroids_num <= 3:
            if self.debug:
                print("Collinearity test requires at least 3 centroids!")
                print()
            return None
        # Sort in case the input centroids not sorted (sanity check)
        id_centroids = sorted(enumerate(centroids), key=lambda elem: math.atan2(elem[1][1], elem[1][0]), reverse=True)
        centroids = [c for i, c in id_centroids]
        for i in range(centroids_num-2):
            x1, y1 = centroids[i][0], centroids[i][1]
            x2, y2 = centroids[i + 1][0], centroids[i + 1][1]
            x3, y3 = centroids[i + 2][0], centroids[i + 2][1]
            dist1 = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            dist2 = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
            triangle_area = 0.5 * abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
            # The formula is basically half of determinant value of the following:
            # x1 x2 x3
            # y1 y2 y3
            # 1   1  1
            # The above formula is derived from shoelace formula
            if self.debug:
                print("Triangle Area: {}".format(triangle_area))
                print("Distance 1:    {}".format(dist1))
                print("Distance 2:    {}".format(dist2))
                print()
            # Collinearity Test:
            # 1. Triangle formed by 3 bay centroids should have area less than a threshold
            # 2. Consecutive bay centroid distance should be larger than a threshold
            if triangle_area < 8 and 8 > dist1 > 5 and 8 > dist2 > 5:
                if self.debug:
                    print("Collinearity test passed!")
                    print()
                ids = [id_centroids[i][0], id_centroids[i + 1][0], id_centroids[i + 2][0]]
                return ids
        ids = None
        return ids

    def average_normal_angle(self, centroids, ids):
        '''
        Compute average normal angle of point cloud cluster centroids.

        :param centroids: Point cloud cluster centroids
        :param ids      : Point cloud cluster centroid IDs

        :return theta_avg: Average angle of all point normals within a cluster
        '''
        points = [(centroids[i][0], centroids[i][1]) for i in ids]
        if len(points) <= 1:
            if self.debug:
                print("Computing average normal angle requires at least 2 centroids!")
                print()
            return 0
        theta_sum = 0
        counter = 0
        for p1, p2 in combinations(points, 2):
            theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            theta = theta - math.pi / 2 if theta > 0 else theta + math.pi / 2
            theta_sum += theta
            counter += 1
        theta_avg = theta_sum / counter if counter != 0 else 0
        return theta_avg

################################################################################

class BayDetector:
    '''
    Detects docking bays by making use of `ColoredRegionDetector` and `DockDetector`.
    '''
    def __init__(self, colors, debug=False):
        self.dock_detector           = DockDetector(debug=debug) # DockDetector class instance
        self.colored_region_detector = ColoredRegionDetector(colors, debug=debug) # ColoredRegionDetector class instance
        self.cv_bridge               = CvBridge() # CvBridge object
        self.color                   = None # Color of target docking bay placard symbol
        self.shape                   = None # Shape of target docking bay placard symbol
        self.debug                   = debug # Flag to enable/disable bay detector debug messages

    def camera_callback(self, msg):
        # Try converting ROS Image message to OpenCV image
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(error)
            print()

    def detect(self, cloud=None):
        '''
        Detect docking bays and identify the target docking bay.

        :param cloud: Updated point cloud

        :return target_bay_pose, all_bay_poses: Pose of the target bay, Poses of all the bays
        '''
        target_bay_pose = None
        all_bay_poses = self.dock_detector.filter_bays_by_plane_normals(cloud)
        bay_num = len(all_bay_poses)
        if bay_num != 3:
            if self.debug:
                print("Plane filter did not detect 3 bays, trying line filter...")
                print()
            all_bay_poses = self.dock_detector.filter_bays_by_collinear_centroids(cloud)
            bay_num = len(all_bay_poses)
            if bay_num == 0:
                if self.debug:
                    print("Line filter did not detect any bays!")
                    print()
                return target_bay_pose, all_bay_poses
        if self.debug:
            print("Plane filter detected {} bays.".format(bay_num))
            print()
        color_shape_to_centroid = self.sort_bay_symbols()
        cs_num = len(color_shape_to_centroid)
        if cs_num == 0:
            if self.debug:
                print("No blobs detected, no need to continue...")
                print()
            return target_bay_pose, all_bay_poses
        if self.debug:
            print("Number of blobs: {}".format(cs_num))
            print("Number of bays:  {}".format(bay_num))
            print()
        if self.color == "Random" or self.shape == "Random":
            print("Unspecified docking bay placard color or shape, docking to a random bay...")
            idx = numpy.random.randint(3)
            print("Bay Number: {}".format(idx+1))
            target_bay_pose = all_bay_poses[idx]
            print("Bay Pose: {}".format(target_bay_pose))
            print()
            return target_bay_pose, all_bay_poses
        if cs_num == bay_num:
            if self.debug:
                print("Number of detected blobs matched the bay count!")
                print()
            for idx, cs in enumerate(color_shape_to_centroid.keys()):
                # According to task description, each bay has unique color and shape
                # Since shape detection is unstable, just use color
                if cs[0] == self.color:
                    print("Docking bay placard color and shape matched, docking to the target bay...")
                    print("Placard Symbol Color: {}".format(self.color))
                    shape = "Rectangle" if self.shape == "Horizontal Rectangle" else self.shape
                    print("Placard Symbol Shape: {}".format(shape))
                    print("Bay Number: {}".format(idx+1))
                    target_bay_pose = all_bay_poses[idx]
                    print("Bay Pose: {}".format(target_bay_pose))
                    print()
                    return target_bay_pose, all_bay_poses
            if cs_num == 3:
                print("Docking bay placard color or shape did not match, docking to a random bay...")
                idx = numpy.random.randint(3)
                print("Bay Number: {}".format(idx+1))
                target_bay_pose = all_bay_poses[idx]
                print("Bay Pose: {}".format(target_bay_pose))
                print()
                return target_bay_pose, all_bay_poses
            if self.debug:
                print("Cannot find target color and shape!")
        else:
            if self.debug:
                print("Number of detected blobs did not match the bay count!")
        return target_bay_pose, all_bay_poses

    def target_bay_symbol(self, target_color, target_shape):
        '''
        Load color and shape of target docking bay placard symbol.

        :param target_color: Color of target docking bay placard symbol
        :param target_shape: Shape of target docking bay placard symbol

        :return: None
        '''
        self.color = target_color
        self.shape = target_shape

    def sort_bay_symbols(self):
        '''
        Sort docking bay placard symbols (colors and shapes) from left to right.

        :param: None

        :return sorted_bay_symbols: Docking bay placard symbols (colors and shapes) sorted from left to right.
        '''
        if self.image is None: # Sanity check
            return {}
        image = self.image
        image_weather = self.image
        self.colored_region_detector.detect(image, image_weather)
        color_shape_to_centroid = self.colored_region_detector.color_shape_to_centroid()
        # Sort by centroid X-coordinate value (i.e. from left to right)
        sorted_bay_symbols = {k: v for k, v in sorted(color_shape_to_centroid.items(), key=lambda item: item[1][0])}
        if self.debug:
            print("Sorted Colored Shapes:")
            print(sorted_bay_symbols)
            print()
        return sorted_bay_symbols

    def plane_angle_and_cloud(self):
        '''
        Compute angle and point cloud of the dock plane.

        :param: None

        :return theta, inlier_cloud: Angle of dock plane, Point cloud of dock plane
        '''
        plane_model, inlier_cloud = self.dock_detector.filter_plane()
        if plane_model is not None and inlier_cloud is not None:
            [a, b, c, d] = plane_model
            if self.debug:
                print("Plane Model: {}, {}, {}, {}".format(a, b, c, d))
                print()
            # Make sure gradient of plane is basically horizontal (i.e. dock plane is vertical)
            theta = math.atan2(b, a) if abs(c) < 0.1 else None
            return theta, inlier_cloud
        if self.debug:
            print("Plane filter failed!")
            print()
        theta = inlier_cloud = None
        return theta, inlier_cloud

################################################################################
