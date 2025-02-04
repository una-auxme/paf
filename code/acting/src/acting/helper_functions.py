"""
Helper functions for calculations
inspired by: PAF 2 WS 20/21 (Acting package)
Trajectory-Interpolation Functions
Source: https://github.com/ll7/paf21-1
"""

import math
from math import dist as euclid_dist, floor, sqrt, sin, cos
from typing import List, Tuple
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation
import rospy


def vectors_to_angle_abs(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Returns the angle (radians) between the two given vectors
    :param x1: v1[x]
    :param y1: v1[y]
    :param x2: v2[x]
    :param y2: v2[y]
    :return: angle between v1 and v2
    """
    v1 = np.array([x1, y1])
    v2 = np.array([x2, y2])

    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    alpha = math.acos(cos_angle)
    return alpha


def vector_angle(x1: float, y1: float) -> float:
    """
    Returns the angle (radians) of a given vectors
    :param x1: v1[x]
    :param y1: v1[y]
    :return: angle between v1 and x-axis [-pi/2, pi/2]
    """
    # v_0 is a vector parallel to the x-axis
    l_v = math.sqrt(x1**2 + y1**2)
    x_0 = x1 + l_v
    y_0 = 0
    # return the angle between the 2 vectors
    alpha = vectors_to_angle_abs(x_0, y_0, x1, y1)
    # check if the angle should be negative
    if y1 < 0:
        sign = -1
    else:
        sign = 1
    return alpha * sign


def vector_to_direction(x1, y1, x2, y2) -> float:
    """
    Returns the direction (angle to y-axis) of a vector.
    :param x1: tail of the vector [x]
    :param y1: tail of the vector [y]
    :param x2: head of the vector [x]
    :param y2: head of the vector [y]
    :return: direction of the vector
    """
    theta = math.atan(x2 - x1 / y2 - y1)
    return math.degrees(theta)


def quaternion_to_heading(x: float, y: float, z: float, w: float) -> float:
    """
    Translates quaternion to euler heading.
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler heading of the given quaternion
    """
    quaternion = (x, y, z, w)
    rot = Rotation.from_quat(quaternion)
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler[2]


def heading_to_quaternion(heading: float) -> Tuple[float, float, float, float]:
    """
    Translates euler heading to quaternion
    :param heading: euler heading
    :return:
    """
    rot = Rotation.from_euler("xyz", (0, 0, heading), degrees=True)
    quat = rot.as_quat()
    return quat[0], quat[1], quat[2], quat[3]


def calc_path_yaw(path: Path, idx: int) -> float:
    """
    Calculates the path yaw
    :param path: The path to calculate the yaw on
    :param idx: The pose index
    :return: see description
    """
    if idx >= len(path.poses) - 1:
        raise RuntimeError("no target found")

    point_current: PoseStamped
    point_current = path.poses[idx]
    point_next: PoseStamped
    point_next = path.poses[idx + 1]
    angle = math.atan2(
        point_next.pose.position.y - point_current.pose.position.y,
        point_next.pose.position.x - point_current.pose.position.x,
    )
    return normalize_angle(angle)


def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle to [-pi, pi]
    :param angle: The angle to normalize
    :return: Angle in radian
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_egocar_yaw(pose: PoseStamped) -> float:
    """
    Calculates the yaw of the ego vehicle
    :param pose: The current pose of the ego vehicle
    :return: normalized yaw of the vehicle
    """
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    _, _, yaw = euler_from_quaternion(quaternion)
    return normalize_angle(yaw)


def points_to_vector(
    p_1: Tuple[float, float], p_2: Tuple[float, float]
) -> Tuple[float, float]:
    """
    Create the vector starting at p1 and ending at p2
    :param p_1: Start point
    :param p_2: End point
    :return: Vector from p1 to p2
    """
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def vector_len(vec: Tuple[float, float]) -> float:
    """
    Compute the given vector's length
    :param vec: vector v as a tuple (x, y)
    :return: length of vector v
    """
    return sqrt(vec[0] ** 2 + vec[1] ** 2)


def add_vector(
    v_1: Tuple[float, float], v_2: Tuple[float, float]
) -> Tuple[float, float]:
    """
    Add the two given vectors
    :param v_1: first vector
    :param v_2: second vector
    :return: sum of first and second vector
    """
    """Add the given vectors"""
    return v_1[0] + v_2[0], v_1[1] + v_2[1]


def rotate_vector(vector: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
    """
    Rotate the given vector by an angle
    :param vector: vector
    :param angle_rad: angle of rotation
    :return: rotated angle
    """
    return (
        cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
        sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1],
    )


def linear_interpolation(
    start: Tuple[float, float], end: Tuple[float, float], interval_m: float
) -> List[Tuple[float, float]]:
    """
    Interpolate linearly between start and end,
    with a minimal distance of interval_m between points.
    :param start: starting point
    :param end: target point
    :param interval_m: min distance between interpolated points, if possible
    :return: interpolated list of points between start / end
    """

    distance = euclid_dist(start, end)
    vector = points_to_vector(start, end)

    steps = max(1, floor(distance / interval_m))
    exceeds_interval_cap = distance > interval_m
    step_vector = (
        vector[0] / steps if exceeds_interval_cap else vector[0],
        vector[1] / steps if exceeds_interval_cap else vector[1],
    )

    lin_points = [(start[0], start[1])]
    for i in range(1, steps):
        lin_points.append(
            (start[0] + step_vector[0] * i, start[1] + step_vector[1] * i)
        )

    return lin_points


def _clean_route_duplicates(
    route: List[Tuple[float, float]], min_dist: float
) -> List[Tuple[float, float]]:
    """
    Remove duplicates in the given List of tuples,
    if the distance between them is less than min_dist.
    :param route: list of points that should be cleaned up
    :param min_dist: minimal allowed distance between points
    :return: cleaned up list of points
    """
    cleaned_route = [route[0]]
    for next_p in route[1:]:
        if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
            cleaned_route.append(next_p)
        else:
            print(next_p)
    return cleaned_route


def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5):
    """
    Interpolate the given route with points inbetween,
    holding the specified distance interval threshold.
    :param orig_route: route
    :param interval_m: minimum distance between two points
    :return: interpolated rout
    """

    orig_route = _clean_route_duplicates(orig_route, 0.1)
    route = []
    for index in range(len(orig_route) - 1):
        waypoints = linear_interpolation(
            orig_route[index], orig_route[index + 1], interval_m
        )
        route.extend(waypoints)

    route = route + [orig_route[-1]]
    return route


def generate_path_from_trajectory(trajectory) -> Path:
    path_msg = Path()
    path_msg.header = rospy.Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "hero"
    path_msg.poses = []
    for wp in trajectory:
        pos = PoseStamped()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = "hero"
        pos.pose.position.x = wp[0]
        pos.pose.position.y = wp[1]
        pos.pose.position.z = 0
        pos.pose.orientation.w = 1
        path_msg.poses.append(pos)

    return path_msg
