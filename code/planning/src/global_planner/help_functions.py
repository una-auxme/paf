from typing import Tuple
from math import floor, asin, sqrt, cos, sin, pi
import numpy as np

"""
A module for interpolating routes and other help functions.
Some parts are covered from source: https://github.com/ll7/paf21-1
"""


def euclid_dist(vector1: Tuple[float, float], vector2: Tuple[float, float]):
    """
    Calculate the euclidian distance between two points.
    :param vector1: first x and y coordinate
    :param vector2: second x and y coordinate
    :return: distance value
    """
    point1 = np.array(vector1)
    point2 = np.array(vector2)

    diff = point2 - point1
    sum_sqrt = np.dot(diff.T, diff)
    return np.sqrt(sum_sqrt)


def unit_vector(vector: Tuple[float, float], size: float)\
        -> Tuple[float, float]:
    """
    Calculate the unit vector.
    :param vector: input vector for calculation
    :param size: multiplying size value
    :return: resulting vector
    """
    length = sqrt(vector[0] ** 2 + vector[1] ** 2)
    return size * (vector[0] / length), size * (vector[1] / length)


def perpendicular_vector_right(vector: Tuple[float, float])\
        -> Tuple[float, float]:
    """
    Perpendicular vector on the right side
    :param vector: input vector
    :return: the resulting vector
    """
    x, y = vector
    perp = (-y, x)
    return perp


def perpendicular_vector_left(vector: Tuple[float, float])\
        -> Tuple[float, float]:
    """
    Perpendicular vector on the left side
    :param vector: input vector
    :return: the resulting vector
    """
    x, y = vector
    perp = (y, -x)
    return perp


def add_vector(v_1: Tuple[float, float], v_2: Tuple[float, float]) \
        -> Tuple[float, float]:
    """
    Addition of two vectors
    :param v_1: first vector with x and y coordinate
    :param v_2: second vector with x and y coordinate
    :return: resulting vector
    """
    return v_1[0] + v_2[0], v_1[1] + v_2[1]


def sub_vector(v_1: Tuple[float, float], v_2: Tuple[float, float]) \
        -> Tuple[float, float]:
    """
    Subtraction of two vectors
    :param v_1: first vector with x and y coordinate
    :param v_2: second vector with x and y coordinate
    :return: resulting vector
    """
    return v_1[0] - v_2[0], v_1[1] - v_2[1]


def rotate_vector(vector: Tuple[float, float], angle_rad: float) \
        -> Tuple[float, float]:
    """
    Rotate the given vector by an angle with the rotationmatrix
    :param vector: input vector with x and y coordinate
    :param angle_rad: rotation angle in rad
    :return: resulting vector
    """
    return (cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
            sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1])


def direction_vector(angle_rad: float) -> Tuple[float, float]:
    """
    Retrieve the unit vector representing the given direction
    :param angle_rad: rotation angle in rad
    :return: resulting vector
    """
    return (cos(angle_rad), sin(angle_rad))


def scale_vector(vector: Tuple[float, float], new_len: float) \
        -> Tuple[float, float]:
    """
    Amplify the length of the given vector
    :param vector: input vector with x and y coordinate
    :param new_len: length for vector scaling
    :return: resulting vector
    """
    old_len = vector_len(vector)
    if old_len == 0:
        return (0, 0)
    scaled_vector = (vector[0] * new_len / old_len,
                     vector[1] * new_len / old_len)
    return scaled_vector


def vector_len(vec: Tuple[float, float]) -> float:
    """
    Compute the given vector's length
    :param vec: input vector with x and y coordinate
    :return: length of the vector
    """
    return sqrt(vec[0]**2 + vec[1]**2)


def points_to_vector(p_1: Tuple[float, float], p_2: Tuple[float, float]) \
        -> Tuple[float, float]:
    """
    Create the vector starting at p1 and ending at p2
    :param p_1: first input vector
    :param p_2: second input vector
    :return: resulting vector
    """
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def end_of_circular_arc(start_point: Tuple[float, float], angle: float,
                        length: float, radius: float) -> Tuple[float, float]:
    """
    Compute the end of a circular arc
    :param start_point: starting point with x and y coordinate
    :param angle: angle value in rad
    :param length: length of the arc
    :param radius: radius of the arc
    :return: endpoint of the arc
    """
    # determine the length of |start, end|
    alpha = length / radius
    diff_vec = scale_vector(direction_vector(alpha), radius)
    dist_start_end = euclid_dist(diff_vec, (radius, 0))

    # determine the direction of |start, end| and apply it
    dir_start_end = direction_vector(alpha / 2 + angle)

    # apply vector |start --> end| to the start point to retrieve the end point
    diff_vec = scale_vector(dir_start_end, dist_start_end)
    return add_vector(start_point, diff_vec)


def circular_interpolation(start: Tuple[float, float],
                           end: Tuple[float, float],
                           arc_radius: float):
    """
    Interpolate points between start / end point
    on top of the circular arc given by the arc radius
    :param start: starting point with x and y coordinate
    :param end: ending point with x and y coordinate
    :param arc_radius: arc radius
    :return: interpolated points
    """

    step_size = 2.0
    sign = -1 if arc_radius < 0 else 1
    arc_radius = abs(arc_radius)

    # determine the circular angle of
    # the arc
    angle = asin((euclid_dist(start, end) / 2) / arc_radius) * 2

    # construct the mid-perpendicular of |start, end| to determine the
    # circle's center
    conn_middle = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    center_offset = sqrt(pow(arc_radius, 2) - pow(euclid_dist(start, end) /
                                                  2, 2))
    mid_perpend = rotate_vector(points_to_vector(start, end), pi/2 * sign)
    circle_center = add_vector(conn_middle, scale_vector(mid_perpend,
                                                         center_offset))

    # partition the arc into steps (-> interpol. geometries)
    arc_circumference = arc_radius * angle
    # (r * 2 pi) * (angle / 2 pi)
    num_steps = int(arc_circumference / step_size) + 1  # each step < step size

    # compute the interpolated points on the circle arc
    vec = points_to_vector(circle_center, start)
    rot_angles = [angle * (i / num_steps) for i in range(num_steps+1)]
    points = [add_vector(circle_center, rotate_vector(vec, rot * sign))
              for rot in rot_angles]

    return points


def linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                         interval_m: float):
    """
    Interpolate linearly between the given start / end point
    by putting points according to the interval specified
    :param start: starting point with x and y coordinate
    :param end: ending point with x and y coordinate
    :param interval_m: interval for number of points
    :return: interpolated points
    """

    distance = euclid_dist(start, end)
    vector = (end[0] - start[0], end[1] - start[1])

    steps = max(1, floor(distance / interval_m))
    exceeds_interval = distance > interval_m
    step_vector = (vector[0] / steps if exceeds_interval else vector[0],
                   vector[1] / steps if exceeds_interval else vector[1])

    lin_points = [(start[0] + step_vector[0] * i,
                   start[1] + step_vector[1] * i) for i in range(steps)]
    lin_points.append(end)
    return lin_points
