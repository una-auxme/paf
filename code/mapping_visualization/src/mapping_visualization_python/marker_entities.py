#!/usr/bin/env python

from entity import Entity
from entity import Rectangle, Circle

from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler


def entity_to_marker(entity: Entity, marker: Marker) -> None:
    if isinstance(entity.shape, Rectangle):
        marker.type = Marker.CUBE
        marker.scale.x = entity.shape.length
        marker.scale.y = entity.shape.width

    elif isinstance(entity.shape, Circle):
        marker.type = Marker.CYLINDER
        marker.scale.x = entity.shape.radius
        marker.scale.y = entity.shape.radius

    marker.pose.position.x = entity.shape.translation.x
    marker.pose.position.y = entity.shape.translation.y
    (
        marker.pose.orientation.x,
        marker.pose.orientation.y,
        marker.pose.orientation.z,
        marker.pose.orientation.w,
    ) = quaternion_from_euler(0, 0, entity.shape.rotation)

    marker.pose.position.z = 0.0
    marker.scale.z = 1.0
    marker.color.a = 0.5

    if entity.is_collider:
        marker.color.r = 255
        marker.color.g = 0
        marker.color.b = 0
    else:
        marker.color.r = 0
        marker.color.g = 255
        marker.color.b = 0
