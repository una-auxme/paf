from typing import List, Optional, Tuple

import shapely
import shapely.ops
import numpy as np
import math

from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Pose
from mapping_common.transform import Transform2D, Point2, Vector2
from shapely.geometry import Polygon, LineString


def curve_to_polygon(line: shapely.LineString, width) -> shapely.Polygon:
    """Creates a polygon with a specified width around a given curve.

    Parameters:
    - points (list of tuple): A list of (x, y) coordinates representing the curve.
    - width (float): The width of the polygon along the curve.

    Returns:
    - Polygon: A Shapely Polygon representing the widened curve."""

    if len(line.coords) < 2:
        raise ValueError("At least two points are required to define a curve.")
    if width <= 0:
        raise ValueError("Width must be a positive value.")

    # Create a LineString from the given points
    curve = LineString(line)

    # Create a buffer around the curve to form a polygon with the given width
    polygon = curve.buffer(
        width / 2, cap_style="round", join_style="round", quad_segs=2
    )

    return polygon


def split_line_at(
    line: shapely.LineString, distance: float
) -> Tuple[Optional[LineString], Optional[LineString]]:
    if len(line.coords) < 2:
        return (None, None)
    if distance <= 0 or math.isclose(0.0, distance):
        return (None, line)

    coords_array = np.array(line.coords)

    coords_0: List[shapely.Point] = []
    coords_1: List[shapely.Point] = []
    current_dist: float = 0.0
    coords_1_start_idx: int = 1
    # First build the line before the split
    for idx in range(1, len(line.coords)):
        p0: Point2 = Point2.new(coords_array[idx - 1, 0], coords_array[idx - 1, 1])
        p1: Point2 = Point2.new(coords_array[idx, 0], coords_array[idx, 1])
        coords_0.append(p0.to_shapely())

        v: Vector2 = p0.vector_to(p1)
        segment_length: float = v.length()
        end_dist: float = current_dist + segment_length

        if math.isclose(end_dist, distance):
            # Split point is p1
            s_p = p1.to_shapely()
            coords_0.append(s_p)
            coords_1.append(s_p)
            coords_1_start_idx = idx + 1
            break
        elif end_dist > distance:
            # Split point is between p0 and p1
            remaining_dist = distance - current_dist
            p_s = p0 + (v.normalized() * remaining_dist)
            s_p = p_s.to_shapely()
            coords_0.append(s_p)
            coords_1.append(s_p)
            coords_1_start_idx = idx
            break
        current_dist = end_dist
    # Now build the line after the split
    for idx in range(coords_1_start_idx, len(line.coords)):
        coords_1.append(shapely.Point(coords_array[idx]))

    line0 = shapely.LineString(coords_0) if len(coords_0) > 1 else None
    line1 = shapely.LineString(coords_1) if len(coords_1) > 1 else None
    return (line0, line1)


def clamp_line(
    line: shapely.LineString,
    start_distance: float = 0.0,
    end_distance: Optional[float] = None,
) -> Optional[shapely.LineString]:
    if len(line.coords) < 2:
        return None
    _, after = split_line_at(line, start_distance)
    if after is None:
        return None
    if end_distance is None:
        return after
    before, _ = split_line_at(after, end_distance)
    return before


def build_trajectory(
    global_trajectory: NavPath,
    global_hero_transform: Transform2D,
    max_length: Optional[float] = None,
    current_wp_idx: int = 0,
    max_wp_count: Optional[int] = None,
    centered: bool = False,
) -> Optional[shapely.LineString]:
    """Builds a local trajectory centered on the global_hero_pos

    Must not be used for navigating, but can be used for collision avoidance / acc
    """
    points = []
    poses_view = (
        global_trajectory.poses[current_wp_idx:]
        if max_wp_count is None
        else global_trajectory.poses[current_wp_idx : current_wp_idx + max_wp_count]
    )
    for pose in poses_view:
        pose: Pose = pose.pose
        points.append(Point2.new(pose.position.x, pose.position.y).to_shapely())
    global_line = shapely.LineString(points)
    hero_pt = global_hero_transform.translation().point()
    hero_pt_s = shapely.Point(hero_pt.x(), hero_pt.y())
    hero_dist: float = global_line.line_locate_point(other=hero_pt_s)
    end_dist: Optional[float] = None if max_length is None else hero_dist + max_length
    clamped = clamp_line(global_line, hero_dist, end_dist)
    if clamped is None:
        return None

    if centered:
        rot_angle: float = global_hero_transform.rotation()
        start_x, start_y = clamped.coords[0]
        global_hero_transform = Transform2D.new_rotation_translation(
            rot_angle, Vector2.new(start_x, start_y)
        )

    return global_hero_transform.inverse() * clamped


def build_trajectory_shape(
    global_trajectory: NavPath,
    global_hero_transform: Transform2D,
    width: float = 1.0,
    max_length: Optional[float] = None,
    current_wp_idx: int = 0,
    max_wp_count: Optional[int] = None,
    centered: bool = False,
) -> Optional[shapely.Polygon]:
    """Calculates the closest entity on the given trajectory. Transforms
    trajectory world coordinates into map coordinates based on hero position.

    Args:
        trajectory (np array of x,y tuples): A np array of
        (x, y) coordinates representing the
        planned trajectory.
        hero_pos (x, y): The world coordinates of the hero car.
        hero_heading (float): The current heading of the hero car.
        width (float): The desired width of the curved polygon.

    Returns:
        Optional[Entity]: The closest entity
    """
    line = build_trajectory(
        global_trajectory,
        global_hero_transform,
        max_length,
        current_wp_idx,
        max_wp_count,
        centered,
    )
    if line is None:
        return None

    curve = curve_to_polygon(line, width)
    return curve


def project_plane(start_point, size_x, size_y) -> shapely.Polygon:
    """
    Projects a rectangular plane starting from (0, 0) forward in the x-direction.

    Parameters:
    - start_point(float, float): Starting point tuple from which
    the rectangle is constructed
    - size_x (float): Length of the plane along the x-axis.
    - size_y (float): Width of the plane along the y-axis.

    Returns:
    - Polygon: A Shapely Polygon representing the plane.
    """
    x, y = start_point

    points = [
        (x, y),
        (x + size_x, y),
        (x + size_x, y + size_y),
        (x, y + size_y),
        (x, y),
    ]

    return Polygon(points)
