from typing import Optional, Tuple

import shapely
import shapely.ops
import numpy as np
import numpy.typing as npt
import math

from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Pose
from mapping_common.transform import Transform2D, Point2, Vector2
from mapping_common.entity import Entity
from mapping_common.shape import Polygon
from shapely.geometry import LineString


def curve_to_polygon(line: shapely.LineString, width: float) -> shapely.Polygon:
    """Creates a polygon with a specified width around a given line.

    Args:
        line (shapely.LineString)
        width (float): Width of the result

    Returns:
        shapely.Polygon
    """

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
    """Splits line at the given distance from the line start

    Args:
        line (shapely.LineString): Line to split
        distance (float)

    Returns:
        Tuple[Optional[LineString], Optional[LineString]]:
            (before, after) Tuple: Line before and after the split.
            Either of them might be None depending on the split position.
    """
    if len(line.coords) < 2:
        return (None, None)
    if distance <= 0 or math.isclose(0.0, distance):
        return (None, line)

    coords_array: npt.NDArray[np.float64] = np.array(line.coords)
    current_dist: float = 0.0
    coords_0_start_idx: int = 0
    coords_0_end_idx: int = len(line.coords) - 1
    new_split_point: Optional[Point2] = None
    coords_1_start_idx: int = len(line.coords) - 1
    coords_1_end_idx: int = len(line.coords) - 1
    # First build the line before the split
    for idx in range(1, len(line.coords)):
        p0: Point2 = Point2.new(coords_array[idx - 1, 0], coords_array[idx - 1, 1])
        p1: Point2 = Point2.new(coords_array[idx, 0], coords_array[idx, 1])

        v: Vector2 = p0.vector_to(p1)
        segment_length: float = v.length()
        end_dist: float = current_dist + segment_length

        if math.isclose(end_dist, distance):
            # Split point is p1
            coords_0_end_idx = idx
            coords_1_start_idx = idx
            break
        elif end_dist > distance:
            # Split point is between p0 and p1
            remaining_dist = distance - current_dist
            new_split_point = p0 + (v.normalized() * remaining_dist)
            coords_0_end_idx = idx - 1
            coords_1_start_idx = idx
            break
        current_dist = end_dist

    line0 = None
    if coords_0_start_idx < coords_0_end_idx:
        coords0: npt.NDArray[np.float64] = coords_array[
            coords_0_start_idx : coords_0_end_idx + 1
        ]
        if new_split_point is not None:
            coords0 = np.concatenate(
                (coords0, [[new_split_point.x(), new_split_point.y()]])
            )
        line0 = shapely.LineString(coords0)

    # Now build the line after the split
    line1 = None
    if coords_1_start_idx < coords_1_end_idx:
        coords1: npt.NDArray[np.float64] = coords_array[
            coords_1_start_idx : coords_1_end_idx + 1
        ]
        if new_split_point is not None:
            coords1 = np.concatenate(
                ([[new_split_point.x(), new_split_point.y()]], coords1)
            )
        line1 = shapely.LineString(coords1)

    return (line0, line1)


def clamp_line(
    line: shapely.LineString,
    start_distance: float = 0.0,
    end_distance: Optional[float] = None,
) -> Optional[shapely.LineString]:
    """Clamps line based on the two distances from the line start

    Args:
        line (shapely.LineString): Line to clamp
        start_distance (float, optional):
            Distance for the first cut from the line start point. Defaults to 0.0.
        end_distance (Optional[float], optional):
            Distance for the second cut from the original line start point.
            If None only the starting section is cut off. Defaults to None.

    Returns:
        Optional[shapely.LineString]: None if the clamping leaves nothing
    """
    if len(line.coords) < 2:
        return None
    _, after = split_line_at(line, start_distance)
    if after is None:
        return None
    if end_distance is None:
        return after
    assert end_distance >= start_distance
    before, _ = split_line_at(after, end_distance - start_distance)
    return before


def build_trajectory(
    global_trajectory: NavPath,
    global_hero_transform: Transform2D,
    max_length: Optional[float] = None,
    current_wp_idx: int = 0,
    max_wp_count: Optional[int] = None,
    centered: bool = False,
) -> Optional[shapely.LineString]:
    """Builds a local trajectory line based on the global trajectory
    the and global_hero_transform and returns it as line

    The global_hero_transform can be built with
    mapping_common.map.build_global_hero_transform().

    When centered is true, the trajectory start point will be centered onto (0, 0).
    Centered mode must not be used for navigating,
    but can be used for collision avoidance / acc.

    Args:
        global_trajectory (NavPath): NavPath trajectory in global coordinates
        global_hero_transform (Transform2D): Global Transform2D of the hero
        max_length (Optional[float], optional): Maximum length of the resulting line.
            Defaults to None.
        current_wp_idx (int, optional): Waypoint index for
            very rough clamping of the NavPath. Defaults to 0.
        max_wp_count (Optional[int], optional): Max waypoint index of the NavPath
            for very rough clamping of the NavPath. Defaults to None.
        centered (bool, optional): Centered mode. Defaults to False.

    Returns:
        Optional[shapely.LineString]: Local line based on the trajectory
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
    start_dist_from_hero: Optional[float] = 0.0,
    max_length: Optional[float] = None,
    current_wp_idx: int = 0,
    max_wp_count: Optional[int] = None,
    centered: bool = False,
) -> Optional[shapely.Polygon]:
    """Builds a shape based on the global trajectory
    the and global_hero_transform and returns it as Polygon

    Args:
        global_trajectory (NavPath): NavPath trajectory in global coordinates
        global_hero_transform (Transform2D): Global Transform2D of the hero
        width (float, optional): Width of the trajectory shape. Defaults to 1.0.
        start_dist_from_hero (Optional[float], optional):
            Removes the first meters from the trajectory. Defaults to 0.0.
        max_length (Optional[float], optional):
            Maximum length of the resulting shape. Defaults to None.
        current_wp_idx (int, optional): Waypoint index for
            very rough clamping of the NavPath. Defaults to 0.
        max_wp_count (Optional[int], optional): Max waypoint index of the NavPath
            for very rough clamping of the NavPath. Defaults to None.
        centered (bool, optional): Centered mode. Defaults to False.

    Returns:
        Optional[shapely.Polygon]: Local shape based on the trajectory
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
    if start_dist_from_hero is not None:
        _, after = split_line_at(line, start_dist_from_hero)
        if after is None:
            return None
        line = after

    curve = curve_to_polygon(line, width)
    return curve


def project_plane(
    size_x: float, size_y: float, start_point: Optional[Point2] = None
) -> shapely.Polygon:
    """
    Projects a rectangular plane starting from (0, 0) forward in the x-direction.

    Parameters:
    - size_x (float): Length of the plane along the x-axis.
    - size_y (float): Width of the plane along the y-axis.
    - start_point(Optional[Point2], optional):
        Start point in the low y-center of the rectangle.
        Default: Point2.zero()

    Returns:
    - Polygon: A Shapely Polygon representing the plane.
    """
    if start_point is None:
        start_point = Point2.zero()
    x = start_point.x()
    y = start_point.y() - size_y / 2.0

    points = [
        (x, y),
        (x + size_x, y),
        (x + size_x, y + size_y),
        (x, y + size_y),
        (x, y),
    ]

    return shapely.Polygon(points)


def point_along_line_angle(x: float, y: float, angle: float, distance: float) -> Point2:
    """
    Calculates a point along a straight line with a given angle and distance.

    Parameters:
    - x (float): x-coordinate of the original position
    - y (float): y-coordinate of the original position
    - angle (float): Angle of the straight line (in rad)
    - distance (float): Distance along the straight line (positive or negative)
    Returns:
        Point2(x,y): x-y-coordinates of new point as Point2
    """
    x_new = x + distance * np.cos(angle)
    y_new = y + distance * np.sin(angle)

    return Point2.new(x_new, y_new)


def create_lane_box(
    y_axis_line: LineString,
    lane_close_hero: Entity,
    lane_further_hero: Entity,
    lane_pos: int,
    lane_length: float,
    lane_transform: float,
    reduce_lane: float,
) -> shapely.Geometry:
    """helper function to create a lane box entity

    Args:
        y_axis_line (LineString): check shape y-axis line
        lane_close_hero (Entity): the lane marking entity that is closer to the car
        lane_further_hero (Entity): the lane marking entity that is further away
            from the car
        lane_pos (int): to check if the lane is on the left or right side of the car
        lane_length (float): length of the lane box
        lane_transform (float): transform of the lane box
        reduce_lane (float): reduce the lane

    Returns:
        lane_box (Geometry): created lane box shape
    """
    close_rotation = lane_close_hero.transform.rotation()
    further_rotation = lane_further_hero.transform.rotation()

    # use intersection of y-axis with lanemarks as helper coordinates for lane boxes
    lane_box_intersection_close = y_axis_line.intersection(
        lane_close_hero.shape.to_shapely(lane_close_hero.transform)
    )
    lane_box_center_close = [
        lane_box_intersection_close.centroid.x,
        lane_box_intersection_close.centroid.y,
    ]
    lane_box_intersection_further = y_axis_line.intersection(
        lane_further_hero.shape.to_shapely(lane_further_hero.transform)
    )
    lane_box_center_further = [
        lane_box_intersection_further.centroid.x,
        lane_box_intersection_further.centroid.y,
    ]

    # get width of the lane box for checking if reduce_lane parameter
    # is fitting for this width. if not, reduce reduce_lane parameter
    lane_box_width = abs(lane_box_center_close[1] - lane_box_center_further[1])
    reduce_lane = min(reduce_lane, lane_box_width)

    # Get half lane box length for calculating lane box shape
    lane_length_half = lane_length / 2

    # Calculating edge points of the lane box shape
    lane_box_close_front = point_along_line_angle(
        lane_box_center_close[0] + lane_transform,
        lane_box_center_close[1] + lane_pos * reduce_lane / 2,
        close_rotation,
        lane_length_half,
    )
    lane_box_close_back = point_along_line_angle(
        lane_box_center_close[0] + lane_transform,
        lane_box_center_close[1] + lane_pos * reduce_lane / 2,
        close_rotation,
        -lane_length_half,
    )
    lane_box_further_front = point_along_line_angle(
        lane_box_center_further[0] + lane_transform,
        lane_box_center_further[1] - lane_pos * reduce_lane / 2,
        further_rotation,
        lane_length_half,
    )
    lane_box_further_back = point_along_line_angle(
        lane_box_center_further[0] + lane_transform,
        lane_box_center_further[1] - lane_pos * reduce_lane / 2,
        further_rotation,
        -lane_length_half,
    )

    lane_box_shape = Polygon(
        [
            lane_box_close_front,
            lane_box_further_front,
            lane_box_further_back,
            lane_box_close_back,
            lane_box_close_front,
        ],
        Transform2D.identity(),
    )

    return lane_box_shape.to_shapely()
