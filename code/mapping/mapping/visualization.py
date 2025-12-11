from typing import List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from mapping_interfaces.msg import Map as MapMsg
from rcl_interfaces.msg import (
    ParameterDescriptor,
    IntegerRange,
)

from mapping_common.entity import FlagFilter
from mapping_common.map import Map

from paf_common.parameters import update_attributes

MARKER_NAMESPACE: str = "map"


class Visualization(Node):
    """The visualization for the intermediate layer.

    This Node will publish the marker array composed of the different entities.
    """

    def __init__(self):
        super().__init__("mapping_visualization")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.map_topic = (
            self.declare_parameter("map_topic", "/paf/hero/mapping/init_data")
            .get_parameter_value()
            .string_value
        )

        self.show_meta_markers = (
            self.declare_parameter(
                "show_meta_markers",
                True,
                descriptor=ParameterDescriptor(
                    description="Show meta information for entities",
                ),
            )
            .get_parameter_value()
            .bool_value
        )

        self.show_tracking_info = (
            self.declare_parameter(
                "show_tracking_info",
                False,
                descriptor=ParameterDescriptor(
                    description="Show tracking information for entities",
                ),
            )
            .get_parameter_value()
            .bool_value
        )

        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, "/paf/hero/mapping/marker_array", qos_profile=1
        )

        self.create_map_sub()

        self.value_map = {-1: False, 0: None, 1: True}
        flag_descriptor = ParameterDescriptor(
            description="Select 0 for ANY, -1 for ISNOT and +1 for IS "
            "in order to filter",
            integer_range=[IntegerRange(from_value=-1, to_value=1)],
        )
        for flag in [
            "flag_motion",
            "flag_collider",
            "flag_tracked",
            "flag_stopmark",
            "flag_lanemark",
            "flag_ignored",
            "flag_hero",
        ]:
            value = (
                self.declare_parameter(flag, 0, descriptor=flag_descriptor)
                .get_parameter_value()
                .integer_value
            )
            setattr(self, flag, value)

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        old_map_topic = self.map_topic
        result = update_attributes(self, params)
        if old_map_topic != self.map_topic:
            self.destroy_subscription(self.map_sub)
            self.create_map_sub()
        return result

    def create_map_sub(self):
        self.map_sub = self.create_subscription(
            topic=self.map_topic,
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

    def map_callback(self, data: MapMsg):
        map = Map.from_ros_msg(data)

        marker_array = MarkerArray()
        marker_array.markers.append(self.create_deleteall_marker())

        marker_timestamp = self.get_clock().now().to_msg()
        markers = []
        filter = FlagFilter(
            has_motion=self.value_map.get(self.flag_motion),
            is_collider=self.value_map.get(self.flag_collider),
            is_tracked=self.value_map.get(self.flag_tracked),
            is_stopmark=self.value_map.get(self.flag_stopmark),
            is_lanemark=self.value_map.get(self.flag_lanemark),
            is_ignored=self.value_map.get(self.flag_ignored),
            is_hero=self.value_map.get(self.flag_hero),
        )
        for entity in map.entities:
            if not entity.matches_filter(filter):
                continue
            markers.append(entity.to_marker(self.show_tracking_info))
            if not self.show_meta_markers:
                continue
            markers.extend(entity.get_meta_markers(self.show_tracking_info))

        for id, marker in enumerate(markers):
            marker.header.stamp = marker_timestamp
            marker.header.frame_id = "hero"
            marker.ns = MARKER_NAMESPACE
            marker.id = id + 1000
            marker.lifetime = Duration(seconds=2 / 20.0).to_msg()
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def create_deleteall_marker(self) -> Marker:
        """Creates a marker that deletes all markers in RViz for this topic.

        Prepend this to the MarkerArray to delete all markers
        before the new ones are displayed.
        """
        return Marker(ns=MARKER_NAMESPACE, action=Marker.DELETEALL)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Visualization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
