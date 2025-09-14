"""Contains functions to easily create debug markers that can be visualized in RViz

**[API documentation](/doc/mapping/generated/mapping_common/markers.md)**

Overview of the main components:
- debug_marker(): Creates a ROS Marker based on different objects
- debug_marker_array(): Creates a ROS MarkerArray
  based on list of ROS Markers
"""

from typing import Optional, List, Tuple, Any
from collections.abc import Sequence

import shapely

from mapping_common.entity import Entity
from mapping_common.shape import Shape2D, MarkerStyle, Polygon, Circle
from mapping_common.transform import Transform2D, Vector2, Point2

from visualization_msgs.msg import Marker, MarkerArray
from transforms3d.euler import euler2quat

from builtin_interfaces.msg import Time as TimeMsg
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration


def debug_marker(
    base: Any,
    frame_id: Optional[str] = "hero",
    position_z: Optional[float] = None,
    transform: Optional[Transform2D] = None,
    offset: Optional[Vector2] = None,
    color: Optional[Tuple[float, float, float, float]] = None,
    scale_z: Optional[float] = None,
) -> Marker:
    """Creates a ROS Marker based on *base*

    Args:
        base (Any): Currently supported: Entity, Shape2D, shapely.Polygon,
            shapely.LineString, Marker, str, Point2, [Point2, Point2] as Arrow
        frame_id (Optional[str], optional): Defaults to "hero".
        position_z (Optional[float], optional): Defaults to None.
            If None, the z position of base will be used.
        transform (Optional[Transform2D], optional): Defaults to None.
            If None, the transform of base will be used.
        offset (Optional[Vector2], optional): Offset Vector.
            Added to the position of the marker. Defaults to None.
        color (Optional[Tuple[float, float, float, float]], optional):
            (r, g, b, a) color tuple. Defaults to (0.5, 0.5, 0.5, 0.5).
        scale_z (Optional[float], optional): Defaults to None.
            If None, the scale of base will be used.
                If the scale.z of base is also 0.0,
                the scale will be set to 1.0 (and 0.3 for str)

    Raises:
        TypeError: If the type of base is unsupported

    Returns:
        Marker: Marker
    """
    if isinstance(base, Entity):
        marker = base.to_marker()
    elif isinstance(base, Shape2D):
        marker = base.to_marker(marker_style=MarkerStyle.LINESTRING)
    elif isinstance(base, shapely.Polygon):
        shape2d = Polygon.from_shapely(base)
        marker = shape2d.to_marker(marker_style=MarkerStyle.LINESTRING)
    elif isinstance(base, shapely.LineString):
        marker = Marker(type=Marker.LINE_STRIP)
        marker.scale.x = 0.05  # Line thickness
        for x, y in base.coords:
            p = Point2.new(x, y)
            marker.points.append(p.to_ros_msg())
    elif isinstance(base, Marker):
        marker = base
    elif isinstance(base, str):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, text=base)
    elif isinstance(base, Point2):
        transform = Transform2D.new_translation(base.vector())
        shape = Circle(radius=0.1)
        marker = shape.to_marker(transform=transform)
    elif isinstance(base, Sequence):
        if len(base) == 2:
            p0 = base[0]
            p1 = base[1]
            if not (isinstance(p0, Point2) and isinstance(p1, Point2)):
                raise TypeError(
                    f"Unsupported debug marker base sequence: '{type(p0)}, {type(p1)}'"
                )
            marker = Marker(type=Marker.ARROW)
            marker.pose.position.x = p0.x()
            marker.pose.position.y = p0.y()
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.3
            marker.points.append(Point2.zero().to_ros_msg())
            marker.points.append(p0.vector_to(p1).point().to_ros_msg())
        else:
            raise TypeError(
                f"Unsupported debug marker base sequence length: '{len(base)}'"
            )
    else:
        raise TypeError(f"Unsupported debug marker base type: '{type(base)}'")

    if frame_id:
        marker.header.frame_id = frame_id
    if position_z:
        marker.pose.position.z = position_z
    if transform:
        transl = transform.translation()

        marker.pose.position.x = transl.x()
        marker.pose.position.y = transl.y()
        (
            marker.pose.orientation.w,
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
        ) = euler2quat(0, 0, transform.rotation())
    if offset:
        marker.pose.position.x += offset.x()
        marker.pose.position.y += offset.y()
    if color is None:
        color = (0.5, 0.5, 0.5, 0.5)
    (
        marker.color.r,
        marker.color.g,
        marker.color.b,
        marker.color.a,
    ) = color
    if scale_z:
        marker.scale.z = scale_z
    elif marker.scale.z == 0.0:
        if marker.type == Marker.TEXT_VIEW_FACING:
            marker.scale.z = 0.3
        else:
            marker.scale.z = 1.0

    return marker


def debug_marker_array(
    namespace: str,
    markers: List[Marker],
    timestamp: TimeMsg,
    lifetime: Optional[DurationMsg] = None,
) -> MarkerArray:
    """Builds a ROS MarkerArray based on *markers*

    Args:
        namespace (str): Namespace of the markers
        markers (List[Marker]): markers
        timestamp (builtin_interfaces.msg.Time): Timestamp of all markers.
        lifetime (Optional[builtin_interfaces.msg.Duration], optional): Marker lifetime.
            Defaults to 0.5.

    Returns:
        MarkerArray: MarkerArray
    """
    if lifetime is None:
        lifetime = Duration(seconds=0.5).to_msg()

    marker_array = MarkerArray(markers=[Marker(ns=namespace, action=Marker.DELETEALL)])
    for id, marker in enumerate(markers):
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = id
        marker.lifetime = lifetime
        marker_array.markers.append(marker)

    return marker_array
