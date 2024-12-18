#!/usr/bin/env python


from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import ros_numpy
import rospy

import random

from typing import List, Optional

from mapping_common.entity import Entity, Flags
from mapping_common.transform import Transform2D, Vector2
from mapping_common.shape import Circle
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

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.new_subscription(
            topic=self.get_param("~lidar_topic", "/carla/hero/LIDAR"),
            msg_type=PointCloud2,
            callback=self.lidar_callback,
            qos_profile=1,
        )
        self.map_publisher = self.new_publisher(
            msg_type=MapMsg,
            topic=self.get_param("~map_init_topic", "/paf/hero/mapping/init_data"),
            qos_profile=1,
        )
        self.rate = 1.0 / 20.0
        self.new_timer(self.rate, self.publish_new_map)

    def lidar_callback(self, data: PointCloud2):
        self.lidar_data = data

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
        map = Map(timestamp=stamp, entities=self.entities_from_lidar())
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)


if __name__ == "__main__":
    name = "mapping_data_integration"
    roscomp.init(name)
    node = MappingDataIntegrationNode(name)
    node.spin()
