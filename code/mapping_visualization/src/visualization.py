#!/usr/bin/env python


import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from visualization_msgs.msg import Marker, MarkerArray
from mapping.msg import Map as MapMsg

from rospy import Publisher
from rospy import Duration

from mapping_common.entity import Entity, FlagFilter
from mapping_common.map import Map

from mapping_visualization.cfg import MappingVisualizationConfig
from dynamic_reconfigure.server import Server

from typing import Optional

MARKER_NAMESPACE: str = "map"


class Visualization(CompatibleNode):
    """The visualization for the intermediate layer.

    This Node will publish the marker array composed of the different entities.
    """

    def __init__(self, args):
        super().__init__("mapping_visualization")

        self.marker_publisher: Publisher = self.new_publisher(
            MarkerArray, "/paf/hero/mapping/marker_array", qos_profile=1
        )

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

        self.value_map = {-1: False, 0: None, 1: True}
        self.flag_motion: Optional[bool] = None
        self.flag_collider: Optional[bool] = None
        self.flag_tracked: Optional[bool] = None
        self.flag_stopmark: Optional[bool] = None
        self.flag_lanemark: Optional[bool] = None
        self.flag_ignored: Optional[bool] = None
        self.flag_hero: Optional[bool] = None

        Server(MappingVisualizationConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config: "MappingVisualizationConfig", level):
        self.flag_motion = self.value_map.get(config["flag_motion"])
        self.flag_collider = self.value_map.get(config["flag_collider"])
        self.flag_tracked = self.value_map.get(config["flag_tracked"])
        self.flag_stopmark = self.value_map.get(config["flag_stopmark"])
        self.flag_lanemark = self.value_map.get(config["flag_lanemark"])
        self.flag_ignored = self.value_map.get(config["flag_ignored"])
        self.flag_hero = self.value_map.get(config["flag_hero"])

        return config

    def map_callback(self, data: MapMsg):
        map = Map.from_ros_msg(data)

        marker_array = MarkerArray()
        marker_array.markers.append(self.create_deleteall_marker())

        marker_timestamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        filter = FlagFilter(
            has_motion=self.flag_motion,
            is_collider=self.flag_collider,
            is_tracked=self.flag_tracked,
            is_stopmark=self.flag_stopmark,
            is_lanemark=self.flag_lanemark,
            is_ignored=self.flag_ignored,
            is_hero=self.flag_hero,
        )
        for id, entity in enumerate(map.entities):
            if not entity.matches_filter(filter):
                continue
            marker_array.markers.append(
                self.create_marker_from_entity(id, entity, marker_timestamp)
            )

        self.marker_publisher.publish(marker_array)

    def create_deleteall_marker(self) -> Marker:
        """Creates a marker that deletes all markers in RViz for this topic.

        Prepend this to the MarkerArray to delete all markers
        before the new ones are displayed.
        """
        return Marker(ns=MARKER_NAMESPACE, action=Marker.DELETEALL)

    def create_marker_from_entity(self, id, entity: Entity, timestamp) -> Marker:
        marker = entity.to_marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = timestamp
        marker.ns = MARKER_NAMESPACE
        marker.id = id
        marker.lifetime = Duration.from_sec(2.0 / 20.0)

        return marker


def main(args=None):
    """Main function to start the node.

    Args:
        args (_type_, optional): Runtime args, do not get processed. Defaults to None.
    """

    roscomp.init("mapping_visualization")

    try:
        node = Visualization(args)
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
