#!/usr/bin/env python3

import math
import numpy
import pymap3d
from tf.transformations import euler_from_quaternion, quaternion_from_euler

################################################################################

def constrain(input, low, high):
    '''
    Constrain input between lower and higher bounds.

    :param input: Input value to be constrained
    :param low  : Lower bound
    :param high : Higher bound

    :return output: Constrained output value
    '''
    if input < low:
      output = low
    elif input > high:
      output = high
    else:
      output = input
    return output

def euclidean_distance(start_pose, goal_pose):
    '''
    Compute the Euclidean distance between two points.

    :param start_pose: Start pose in cartesian coordinates (x, y, yaw)
    :param goal_pose: Goal pose in cartesian coordinates (x, y, yaw)

    :return dist: Euclidean distance between two points in meters
    '''
    dist = math.sqrt((start_pose.position.x - goal_pose.position.x) ** 2 + (start_pose.position.y - goal_pose.position.y) ** 2)
    return dist

def normalize_angle(theta):
    '''
    Normalize angle within [-pi, pi).

    :param theta: Angle in radians

    :return theta_norm: Normalized angle in radians
    '''
    theta_norm = (theta + math.pi) % (2 * math.pi) - math.pi
    return theta_norm

def angle_within_half_plane_range(theta_check, theta_min, theta_max):
    ''''
    Check whether an angle lies within certain half-plane range, all in [-pi, pi).
    Note: Angle on the range edge is not deemed as within it.

    :param theta_check: Angle to be checked in radians in [-pi, pi)
    :param theta_min  : Minimum angle in radians in [-pi, pi)
    :param theta_max  : Maximum angle in radians in [-pi, pi)

    :return result: Boolean flag indicating whether `theta_check` lies within `theta_min` and `theta_max`
    '''
    assert abs(normalize_angle(theta_min - theta_max)) <= math.pi
    if theta_min == theta_max:
        return False
    if theta_check == theta_min or theta_check == theta_max:
        return False
    cur_to_min = normalize_angle(theta_check - theta_min)
    cur_to_max = normalize_angle(theta_check - theta_max)
    cur_quad, start_quad, end_quad = quadrant(theta_check), quadrant(theta_min), quadrant(theta_max)
    result = ((cur_to_min > 0) ^ (cur_to_max > 0)) and ((cur_quad == start_quad) | (cur_quad == end_quad))
    return result

def quadrant(theta):
    '''
    Determine cartesian quadrant of a particular angle.

    :param theta: Angle whose cartesian quadrant is to be determined

    :return quadrant: Cartesian quadrant of the given angle
    '''
    c = math.cos(theta)
    s = math.sin(theta)
    if (c > 0) ^ (s > 0):
        quadrant = 2 if c < 0 else 4
    else:
        quadrant = 1 if c > 0 else 3
    return quadrant

def cartesian_to_polar(start_pose, goal_pose):
    '''
    Convert cartesian pose coordinates to polar pose coordinates.

    :param start_pose: Start pose in cartesian coordinates (x, y, yaw)
    :param goal_pose: Goal pose in cartesian coordinates (x, y, yaw)

    :return rho, alpha, beta: Polar coordinates describing distance to goal, start heading error, goal heading error
    '''
    rho = math.sqrt((start_pose.position.x - goal_pose.position.x) ** 2 + (start_pose.position.y - goal_pose.position.y) ** 2)
    theta = math.atan2(goal_pose.position.y - start_pose.position.y, goal_pose.position.x - start_pose.position.x)
    alpha = normalize_angle(theta - start_pose.orientation.z)
    beta = normalize_angle(normalize_angle(goal_pose.orientation.z - start_pose.orientation.z) - alpha)
    return rho, alpha, beta

def quaternion_to_euler(quat):
    '''
    Convert ROS Quaternion message to Euler angle representation (roll, pitch, yaw).

    :param quat: quaternion

    :return euler: roll=euler[0], pitch=euler[1], yaw=euler[2]
    '''
    q = [quat.x, quat.y, quat.z, quat.w]
    euler = euler_from_quaternion(q)
    return euler

def euler_to_quaternion(roll, pitch, yaw):
    '''
    Convert Euler angle representation (roll, pitch, yaw) to Quaternion vector (x, y, z, w).

    :param roll : Orientation about local X-axis
    :param pitch: Orientation about local Y-axis
    :param yaw  : Orientation about local Z-axis

    :return quat: x=quat[0], y=quat[1], z=quat[2], w=quat[3]
    '''
    quat = quaternion_from_euler(roll, pitch, yaw)
    return quat

def gps_to_enu(lat, lon, alt=0):
    '''
    Convert GPS coordinates (lat, lon, alt) to ENU coordinates (x, y, z).

    :param lat: Latitude in degrees
    :param lon: Longitude in degrees
    :param alt: Altitude in meters

    :return x, y, z: ENU coordinates in meters
    '''
    # Local coordinate origin (Sydney International Regatta Centre, Australia)
    lat0 = -33.724223 # degree North
    lon0 = 150.679736 # degree East
    alt0 = 0 # meters
    enu = pymap3d.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    x = enu[0]
    y = enu[1]
    z = enu[2]
    return x, y, z

def enu_to_gps(x, y, z=0):
    '''
    Convert ENU coordinates (x, y, z) to GPS coordinates (lat, lon, alt).

    :param x: East coordinate in meters
    :param y: North coordinate in meters
    :param z: Up coordinate in meters

    :return lat, lon, alt: GPS coordinates in degrees & meters
    '''
    # Local coordinate origin (Sydney International Regatta Centre, Australia)
    lat0 = -33.724223 # degree North
    lon0 = 150.679736 # degree East
    alt0 = 0 # meters
    gps = pymap3d.enu2geodetic(x, y, z, lat0, lon0, alt0)
    lat = gps[0]
    lon = gps[1]
    alt = gps[2]
    return lat, lon, alt

def heading_to_bearing(heading):
    '''
    Convert given heading in radians to compass bearing in degrees.
    Note: Compass bearing starts at 0 degrees North and increases clockwise.

    :param heading: Heading in radians (0 rad at East, increasing counter-clockwise)

    :return bearing: Compass bearing in degrees (0 deg at North, increasing clockwise)
    '''
    x = math.cos(heading)
    y = math.sin(heading)
    bearing = numpy.rad2deg(math.atan2(x, y))
    if bearing < 0: # If atan2 returns a negative bearing
        bearing += 360
    return bearing

def bearing_to_heading(bearing):
    '''
    Convert given compass bearing in degrees to heading in radians.
    Note: Compass bearing starts at 0 degrees North and increases clockwise.

    :param bearing: Compass bearing in degrees (0 deg at North, increasing clockwise)

    :return heading: Heading in radians (0 rad at East, increasing counter-clockwise)
    '''
    bearing = numpy.deg2rad(bearing)
    x = math.cos(bearing)
    y = math.sin(bearing)
    heading = math.atan2(x, y)
    if heading < 0: # If atan2 returns a negative heading
        heading += 2 * math.pi
    return heading

def local_to_global_tf(x_ref, y_ref, theta_ref, x_loc, y_loc, theta_loc):
    '''
    Transform pose of target object (described w.r.t. reference object) to global frame.

    :param x_ref    : X position of reference object w.r.t. global frame (i.e. global posotion)
    :param y_ref    : Y position of reference object w.r.t. global frame (i.e. global posotion)
    :param theta_ref: Orientation of reference object w.r.t. global frame (i.e. global posotion)
    :param x_loc    : X position of target object w.r.t. reference object (i.e. local posotion)
    :param y_loc    : Y position of target object w.r.t. reference object (i.e. local posotion)
    :param theta_loc: Orientation of target object w.r.t. reference object (i.e. local posotion)

    :return x_global, y_global, theta_global: Pose of target object w.r.t. global frame (i.e. global pose)
    '''
    ref_obj_glb_tf = numpy.array([[math.cos(theta_ref), -math.sin(theta_ref), x_ref],
                                  [math.sin(theta_ref), math.cos(theta_ref),  y_ref],
                                  [0,                   0,                    1    ]])
    tgt_obj_loc_tf = numpy.array([[x_loc],
                                  [y_loc],
                                  [1    ]])
    tgt_obj_glb_tf = numpy.matmul(ref_obj_glb_tf, tgt_obj_loc_tf)
    x_global, y_global, theta_global = tgt_obj_glb_tf[0][0], tgt_obj_glb_tf[1][0], normalize_angle(theta_ref + theta_loc)
    return x_global, y_global, theta_global

################################################################################
