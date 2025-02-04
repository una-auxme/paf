#!/usr/bin/env python


import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray
from mapping.msg import Map as MapMsg

from rospy import Publisher
from rospy import Duration
import rospy

from mapping_common.entity import Entity, Car, Pedestrian
from mapping_common.map import Map
from mapping_common.transform import Point2

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

        self.filter_is_collider = False
        self.filter_is_tracked = False
        self.filter_is_stopmark = False
        self.filter_is_lanemark = False
        self.filter_is_ignored = False

        self.new_service(SetBool, "vis/set_is_collider", self.set_is_collider_filter)
        self.new_service(SetBool, "vis/set_is_tracked", self.set_is_tracked_filter)
        self.new_service(SetBool, "vis/set_is_stopmark", self.set_is_stopmark_filter)
        self.new_service(SetBool, "vis/set_is_lanemark", self.set_is_lanemark_filter)
        self.new_service(SetBool, "vis/set_is_ignored", self.set_is_ignored_filter)

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

    def map_callback(self, data: MapMsg):
        map = Map.from_ros_msg(data)
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_deleteall_marker())

        marker_timestamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        for id, entity in enumerate(map.entities):
            # TODO: filtering based on flags
            markers = self.create_marker_from_entity(id, entity, marker_timestamp)
            for marker in markers:
                marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def create_deleteall_marker(self) -> Marker:
        """Creates a marker that deletes all markers in RViz for this topic.

        Prepend this to the MarkerArray to delete all markers
        before the new ones are displayed.
        """
        return Marker(ns=MARKER_NAMESPACE, action=Marker.DELETEALL)

    def create_marker_from_entity(self, id, entity: Entity, timestamp) -> list:
        marker = entity.to_marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = timestamp
        marker.ns = MARKER_NAMESPACE
        marker.id = id
        marker.lifetime = Duration.from_sec(2 / 20.0)
        # color marker depending on class, if exists
        if isinstance(entity, Car):
            # [0, 0, 255],  # 10: Vehicles
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            marker.color.a = 0.75

        elif isinstance(entity, Pedestrian):
            # [220, 20, 60],  # 4: Pedestrians
            marker.color.r = 220 / 255
            marker.color.g = 20 / 255
            marker.color.b = 60 / 255
            marker.color.a = 0.75

        # if entity.motion is not None create a velocity vector as arrow marker
        if entity.motion is not None:
            motion_arrow_marker = self.create_motion_arrow_marker(entity, id, timestamp)
            motion_text_marker = self.create_motion_text_marer(entity, id, timestamp)
            return [marker, motion_arrow_marker, motion_text_marker]

        return [marker]

    def create_motion_arrow_marker(self, entity: Entity, id, timestamp) -> Marker:
        m = Marker()
        m.type = Marker.ARROW
        m.header.frame_id = "hero"
        m.header.stamp = timestamp
        m.ns = MARKER_NAMESPACE
        m.action = Marker.ADD
        m.id = id + 1000
        m.lifetime = Duration.from_sec(2 / 20.0)
        m.pose.position.x = entity.transform.translation().x()
        m.pose.position.y = entity.transform.translation().y()
        m.pose.position.z = 0.0
        m.scale.x = 0.1
        m.scale.y = 0.3
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        if entity.motion is not None:
            m.points.append(Point2.zero().to_ros_msg())
            m.points.append(
                (entity.transform * entity.motion.linear_motion).point().to_ros_msg()
            )
            rospy.loginfo(m.points)
        return m

    def create_motion_text_marer(self, entity: Entity, id, timestamp) -> Marker:
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.header.frame_id = "hero"
        text_marker.header.stamp = timestamp
        text_marker.ns = MARKER_NAMESPACE
        text_marker.action = Marker.ADD
        text_marker.id = id + 2000
        text_marker.lifetime = Duration.from_sec(2 / 20.0)
        text_marker.pose.position.x = entity.transform.translation().x()
        text_marker.pose.position.y = entity.transform.translation().y()
        text_marker.pose.position.z = 1.5
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        if entity.motion is not None:
            speed_in_ms = entity.motion.linear_motion.length()
            speed_in_kmh = speed_in_ms * 3.6
            text_marker.text = f"{speed_in_kmh:.2f} km/h"
        return text_marker

    def set_is_collider_filter(self, request: SetBoolRequest):
        self.filter_is_collider = request.data
        self.loginfo(f"Called collider {request.data}")

        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_tracked_filter(self, request: SetBoolRequest):
        self.filter_is_tracked = request.data
        self.loginfo(f"Called tracked {request.data}")

        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_stopmark_filter(self, request: SetBoolRequest):
        self.filter_is_stopmark = request.data
        self.loginfo(f"Called stopmark {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_lanemark_filter(self, request: SetBoolRequest):
        self.filter_is_lanemark = request.data

        self.loginfo(f"Called lanemark {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_ignored_filter(self, request: SetBoolRequest):
        self.filter_is_ignored = request.data

        self.loginfo(f"Called ignored {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response


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
