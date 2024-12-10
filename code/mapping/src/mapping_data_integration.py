from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import ros_numpy
import rospy

from typing import List, Optional

from mapping_common.entity import Entity, Flags
from mapping_common.transform import Transform2D, Vector2
from mapping_common.shape import Circle
from mapping_common.map import Map
from mapping.msg import Map as MapMsg

from sensor_msgs.msg import PointCloud2


class MappingDataIntegrationNode(CompatibleNode):
    lidar_entities: Optional[List[Entity]] = None

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
            topic=self.get_param("~map_init_topic", "/paf/hero/mapping_init_data"),
            qos_profile=1,
        )
        self.rate = 1.0 / 20.0
        self.new_timer(self.rate, self.publish_new_map)

    def lidar_callback(self, data: PointCloud2):
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        shape = Circle(0.15)
        self.lidar_entities = []
        for x, y, z, intensity in coordinates:
            if z < -1.5 or z > 1.0:
                # Ignore street level lidar points and stuff above
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
            self.lidar_entities.append(e)

    def publish_new_map(self, timer_event=None):
        # Make sure we have data for each dataset we are subscribed to
        if self.lidar_entities is None:
            return

        stamp = rospy.get_rostime()
        map = Map(timestamp=stamp, entities=self.lidar_entities)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)


if __name__ == "__main__":
    name = "mapping_data_integration"
    roscomp.init(name)
    node = MappingDataIntegrationNode(name)
    node.spin()
