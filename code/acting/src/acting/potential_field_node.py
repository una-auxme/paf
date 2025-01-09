#!/usr/bin/env python

# import os
# import sys
# from matplotlib import pyplot
# import numpy as np

from ros_compatibility.node import CompatibleNode

# from acting.src.acting.potentialField import PotentialField
from mapping.msg import Entity as EntityMsg
from mapping.msg import Map as MapMsg

from acting.entity import Entity
from acting.map import Map

# from mapping.ext_modules.mapping_common.entity import Entity
# from mapping.ext_modules.mapping_common.map import Map
from rospy import Publisher, Subscriber
import ros_compatibility as roscomp
import rospy
import rosgraph


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

    def __get_entities(self, data: MapMsg):
        self.map = Map.from_ros_msg(data)
        self.loginfo(len(self.map.entities))

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
