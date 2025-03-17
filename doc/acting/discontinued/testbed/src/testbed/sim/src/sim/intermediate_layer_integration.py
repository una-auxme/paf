#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy

from sim.msg import Entities, Entity
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import struct
import numpy as np

from typing import Optional, List, Tuple
from rospy import Publisher


class LidarMock(CompatibleNode):

    def __init__(self):
        super().__init__("lidar_mock")

        self.point_cloud_publisher: Publisher = self.new_publisher(
            msg_type=PointCloud2, topic="carla/hero/LIDAR", qos_profile=10
        )

        self.new_subscription(
            msg_type=Entities,
            topic="entities",
            callback=self._update_entities,
            qos_profile=10,
        )

        self.entity_positions: List[Tuple[float, float]] = []

        self.new_timer(1.0 / 20.0, lambda timer_event: self._publish_pointcloud())

    def _publish_pointcloud(self):
        pointcloud_msg = self.__create_pointcloud2_msg()
        self.point_cloud_publisher.publish(pointcloud_msg)

    def _update_entities(self, entities_msg: Entities):
        self.entity_positions = []
        entity: Entity
        for entity in entities_msg.entities:
            self.entity_positions.append((entity.x, entity.y))

    def __create_pointcloud2_msg(self):
        """
        Create a PointCloud2 message from a list of points.
        :param points: A list of (x, y, z) tuples representing points in the cloud.
        :return: PointCloud2 message.
        """
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "hero"  # Frame in which the points are defined

        # Define the fields of the PointCloud2 message
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=8, datatype=PointField.FLOAT32, count=1
            ),
        ]

        # Pack the points into a binary blob
        point_cloud_data = []
        for point in self.entity_positions:
            x, y = point
            z = 0.5
            intensity = 1.0
            point_cloud_data.append(struct.pack("ffff", x, y, z, intensity))

        point_cloud_blob = b"".join(point_cloud_data)

        pointcloud2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(point_cloud_data),
            fields=fields,
            is_bigendian=False,
            point_step=12,  # 4 bytes per x, y, z
            row_step=12 * len(point_cloud_data),
            data=point_cloud_blob,
            is_dense=True,  # If there are no invalid points
        )

        return pointcloud2_msg


if __name__ == "__main__":
    roscomp.init("intermediate_mock")
    lm = LidarMock()
    lm.spin()
