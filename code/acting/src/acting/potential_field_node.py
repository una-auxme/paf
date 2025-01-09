#!/usr/bin/env python

# import os
# import sys
# from matplotlib import pyplot
# import numpy as np

from ros_compatibility.node import CompatibleNode

# from acting.src.acting.potentialField import PotentialField
from mapping.msg import Map as MapMsg

from acting.entity import Entity
from acting.map import Map

# from mapping.ext_modules.mapping_common.entity import Entity
# from mapping.ext_modules.mapping_common.map import Map
from rospy import Publisher, Subscriber
import ros_compatibility as roscomp
import rospy
import rosgraph
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import matplotlib.pyplot as plt


class Potential_field_node(CompatibleNode):

    def __init__(self):
        # self.potentialField = PotentialField(1, [], (0,0))
        self.entities: list[Entity] = []
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.entities_sub: Subscriber = self.new_subscription(
            msg_type=MapMsg,
            topic="/paf/hero/mapping/init_data",
            callback=self.__get_entities,
            qos_profile=1,
        )

        self.entities_plot_pub: Publisher = self.new_publisher(
            Marker, "/paf/hero/mapping/entities_plot", 1
        )

    def __get_entities(self, data: MapMsg):
        self.map = Map.from_ros_msg(data)
        self.loginfo(len(self.map.entities))
        if len(self.map.entities) > 0:
            x = self.map.entities[0].transform.translation().x()
            y = self.map.entities[0].transform.translation().y()
            self.loginfo(f"{x} {y}")

        self.entities_plot_pub.publish()

    def __plot_entities(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "entities"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for entity in self.map.entities:
            point = Point()
            point.x = entity.transform.translation().x()
            point.y = entity.transform.translation().y()
            point.z = 0
            marker.points.append(point)

        return marker

    def run(self):
        self.loginfo("Potential Field Node Running")

        def loop(timerevent=None):
            pass

        self.new_timer(1, loop)
        self.spin()


def main(args=None):
    """_summary_"""
    roscomp.init("potential_field_node", args=args)

    try:
        node = Potential_field_node()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
