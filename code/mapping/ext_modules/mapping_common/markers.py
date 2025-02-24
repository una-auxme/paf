from typing import Optional, List, Tuple, Any

import shapely

from mapping_common.entity import Entity
from mapping_common.shape import Shape2D, MarkerStyle, Polygon
from mapping_common.transform import Transform2D, Vector2

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler


def debug_marker(
    base: Any,
    frame_id: Optional[str] = "hero",
    position_z: Optional[float] = None,
    transform: Optional[Transform2D] = None,
    offset: Optional[Vector2] = None,
    color: Optional[Tuple[float, float, float, float]] = None,
    scale_z: Optional[float] = None,
) -> Marker:
    """Creates a marker based on *base*

    Args:
        base (Any): Currently supported: Entity, Shape2D, shapely.Polygon, Marker, str
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
        Marker
    """
    if isinstance(base, Entity):
        marker = base.to_marker()
    elif isinstance(base, Shape2D):
        marker = base.to_marker(marker_style=MarkerStyle.LINESTRING)
    elif isinstance(base, shapely.Polygon):
        shape2d = Polygon.from_shapely(base)
        marker = shape2d.to_marker(marker_style=MarkerStyle.LINESTRING)
    elif isinstance(base, Marker):
        marker = base
    elif isinstance(base, str):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, text=base)
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
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, transform.rotation())
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
    timestamp: Optional[rospy.Time] = None,
    lifetime: Optional[rospy.Duration] = None,
) -> MarkerArray:
    """Builds a MArkerArray based on *markers*

    Args:
        namespace (str): Namespace of the markers
        markers (List[Marker])
        timestamp (Optional[rospy.Time], optional): Timestamp of all markers.
            Defaults to None. If None, the current ros time will be used
        lifetime (Optional[rospy.Duration], optional): Marker lifetime.
            Defaults to 0.5.

    Returns:
        MarkerArray
    """
    if lifetime is None:
        lifetime = rospy.Duration.from_sec(0.5)
    if timestamp is None:
        timestamp = rospy.get_rostime()

    marker_array = MarkerArray(markers=[Marker(ns=namespace, action=Marker.DELETEALL)])
    for id, marker in enumerate(markers):
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = id
        marker.lifetime = lifetime
        marker_array.markers.append(marker)

    return marker_array
