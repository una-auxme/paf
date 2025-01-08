#!/usr/bin/env python


from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import ros_numpy
import rospy
from visualization_msgs.msg import MarkerArray
import random
import numpy as np
from typing import List, Optional

from mapping_common.entity import Entity, Flags
from mapping_common.transform import Transform2D, Vector2
from mapping_common.shape import Circle, Rectangle
from mapping_common.map import Map
from mapping.msg import Map as MapMsg

from sensor_msgs.msg import PointCloud2


class MappingDataIntegrationNode(CompatibleNode):
    """Creates the initial map data frame based on all kinds of sensor data

    Sends this map off to Filtering and other consumers (planning, acting)

    This node sends the maps off at a fixed rate.
    (-> It buffers incoming sensor data slightly)
    """

    lidar_data: Optional[PointCloud2] = None
    lidar_marker_data: Optional[MarkerArray] = None

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.new_subscription(
            topic=self.get_param("~lidar_topic", "/carla/hero/LIDAR"),
            msg_type=PointCloud2,
            callback=self.lidar_callback,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param("~marker_topic", "/paf/hero/Lidar/Marker"),
            msg_type=MarkerArray,
            callback=self.lidar_marker_callback,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param("~marker_topic", "/paf/hero/Radar/Marker"),
            msg_type=MarkerArray,
            callback=self.radar_marker_callback,
            qos_profile=1,
        )

        self.map_publisher = self.new_publisher(
            msg_type=MapMsg,
            topic=self.get_param("~map_init_topic", "/paf/hero/mapping/init_data"),
            qos_profile=1,
        )
        self.rate = 1.0 / 20.0
        self.new_timer(self.rate, self.publish_new_map)

    def lidar_marker_callback(self, data: MarkerArray):
        self.lidar_marker_data = data

    def radar_marker_callback(self, data: MarkerArray):
        self.radar_marker_data = data

    def lidar_callback(self, data: PointCloud2):
        self.lidar_data = data

    def entities_from_lidar_marker(self) -> List[Entity]:
        data = self.lidar_marker_data
        if data is None or not hasattr(data, "markers") or data.markers is None:
            rospy.logwarn("No valid marker data received.")
            return []

        lidar_entities = []
        for marker in data.markers:
            if not marker.points:
                rospy.logwarn(f"Skipping empty marker with ID: {marker.id}")
                continue

            width, length = calculate_marker_width_length_2d(marker.points)
            x_center, y_center = calculate_marker_center_2d(marker.points)

            shape = Rectangle(width, length)
            v = Vector2.new(x_center, y_center)
            transform = Transform2D.new_translation(v)

            flags = Flags(is_collider=True)
            e = Entity(
                confidence=1,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=marker.header.stamp,
                flags=flags,
            )
            lidar_entities.append(e)

        return lidar_entities

    def entities_from_radar_marker(self) -> List[Entity]:
        data = self.radar_marker_data
        if data is None or not hasattr(data, "markers") or data.markers is None:
            # Handle cases where data or markers are invalid
            rospy.logwarn("No valid marker data received.")
            return []

        radar_entities = []
        for marker in data.markers:
            if not marker.points:
                rospy.logwarn(f"Skipping empty marker with ID: {marker.id}")
                continue

            width, length = calculate_marker_width_length_2d(marker.points)
            x_center, y_center = calculate_marker_center_2d(marker.points)

            shape = Rectangle(width, length)
            v = Vector2.new(x_center, y_center)
            transform = Transform2D.new_translation(v)

            flags = Flags(is_collider=True)
            e = Entity(
                confidence=1,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=marker.header.stamp,
                flags=flags,
            )
            radar_entities.append(e)

        return radar_entities

    def entities_from_lidar(self) -> List[Entity]:
        if self.lidar_data is None:
            return []

        data = self.lidar_data
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        shape = Circle(0.15)
        lidar_entities = []
        for x, y, z, intensity in coordinates:
            if z < -1.5 or z > 1.0:
                # Ignore street level lidar points and stuff above
                continue
            if random.random() < 0.9:
                # Get rid of points because performance
                continue
            v = Vector2.new(x, y)
            transform = Transform2D.new_translation(v)
            flags = Flags(is_collider=True)
            e = Entity(
                confidence=0.5 * intensity,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=data.header.stamp,
                flags=flags,
            )
            lidar_entities.append(e)

        return lidar_entities

    def publish_new_map(self, timer_event=None):
        # Make sure we have data for each dataset we are subscribed to
        if self.lidar_data is None:
            return

        stamp = rospy.get_rostime()
        lidar_entities = self.entities_from_lidar_marker()
        radar_entities = self.entities_from_radar_marker()
        entities = lidar_entities + radar_entities
        map = Map(timestamp=stamp, entities=entities)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)


def calculate_marker_center_2d(points):
    """
    Calculates the center (x, y) of a 2D bounding box based on marker points.

    Args:
        points (list or numpy.ndarray): List of geometry_msgs/Point objects
        or NumPy array.

    Returns:
        tuple: Center coordinates (x_center, y_center).
    """
    # Convert list of points to NumPy array if necessary
    points_array = (
        np.array([[p.x, p.y] for p in points])
        if not isinstance(points, np.ndarray)
        else points
    )

    # Calculate min and max values
    x_min, x_max = np.min(points_array[:, 0]), np.max(points_array[:, 0])
    y_min, y_max = np.min(points_array[:, 1]), np.max(points_array[:, 1])

    # Calculate the center
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2

    return x_center, y_center


def calculate_marker_width_length_2d(points):
    """
    Calculates the width and length of a 2D bounding box (x, y dimensions only).

    Args:
        points (list or numpy.ndarray): List or array of points with [x, y, z].

    Returns:
        tuple: Width and length of the bounding box.
    """
    # Convert to NumPy array if points is a list
    points_array = (
        np.array([[p.x, p.y] for p in points])
        if not isinstance(points, np.ndarray)
        else points
    )

    # Calculate width and length using min/max on x and y dimensions
    x_min, x_max = np.min(points_array[:, 0]), np.max(points_array[:, 0])
    y_min, y_max = np.min(points_array[:, 1]), np.max(points_array[:, 1])

    width = x_max - x_min
    length = y_max - y_min

    return width, length


if __name__ == "__main__":
    name = "mapping_data_integration"
    roscomp.init(name)
    node = MappingDataIntegrationNode(name)
    node.spin()
