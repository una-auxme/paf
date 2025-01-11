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

import numpy as np
from PIL import Image

DISTANCE_THRESHOLD = 100
RESOLUTION_SCALE = 10


class Potential_field_node(CompatibleNode):

    def __init__(self):
        # self.potentialField = PotentialField(1, [], (0,0))
        self.entities: list[Entity] = []
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.entity_matrix = None

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
        # seems like the uuid of the entities is not implemended yet
        self.entities = self.map.entities_without_hero()
        self.loginfo("Got entities")

    def __filter_entities(self):
        # filter out entities with a larger euclidean distance than DISTANCE_THRESHOLD
        self.loginfo(f" length before filtering {len(self.entities)}")
        self.entities = [
            entity
            for entity in self.entities
            if entity.transform.translation().length() < DISTANCE_THRESHOLD
        ]

        # self.loginfo(f"after first filtering :{len(self.entities)}")

        # filter out duplicates with similar position by filling it into a map
        self.entity_matrix = np.zeros(
            (
                DISTANCE_THRESHOLD * RESOLUTION_SCALE,
                DISTANCE_THRESHOLD * RESOLUTION_SCALE,
            )
        )
        for entity in self.entities:
            x = int(entity.transform.translation().x() * RESOLUTION_SCALE)
            y = int(entity.transform.translation().y() * RESOLUTION_SCALE)
            if self.entity_matrix[x][y] == 0:
                self.entity_matrix[x][y] = 1

        self.loginfo(f"after filtering :{len(self.entities)}")

    def __plot_entities(self):
        self.__filter_entities()

        self.entity_matrix_for_plotting = np.zeros(
            (
                self.entity_matrix.shape[0],
                self.entity_matrix.shape[1],
                3,
            )
        )

        # normalize the entity matrix to 0-255
        self.entity_matrix = self.entity_matrix * 255

        # flip image verically
        self.entity_matrix_for_plotting = np.flipud(self.entity_matrix)

        image = Image.fromarray(self.entity_matrix_for_plotting)
        image = image.convert("RGB")
        image.save("entity_matrix.png")

    def run(self):
        self.loginfo("Potential Field Node Running")

        def loop(timerevent=None, timed=True):
            if timed:
                starttime = rospy.get_time()
                self.loginfo(f"TIMER LOOP START {starttime}")
            self.__filter_entities()
            if timed:
                filtering_start = rospy.get_time()
                self.loginfo(f"TIME TAKEN FOR FILTERING {rospy.get_time()-starttime}")
            self.__plot_entities()
            if timed:
                self.loginfo(
                    f"TIME TAKEN FOR PLOTTING {rospy.get_time()-filtering_start}"
                )
                self.loginfo(f"TIMER TAKEN FOR LOOP {rospy.get_time()-starttime}")

        self.new_timer(0.3, loop)
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
