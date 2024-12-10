from dataclasses import dataclass, field
from typing import List

from genpy.rostime import Time
from std_msgs.msg import Header

from mapping_common.entity import Entity

from mapping import msg


@dataclass
class Map:
    timestamp: Time = Time()
    entities: List[Entity] = field(default_factory=list)

    @staticmethod
    def from_ros_msg(m: msg.Map) -> "Map":
        entities = list(map(lambda e: Entity.from_ros_msg(e), m.entities))
        return Map(timestamp=m.header.stamp, entities=entities)

    def to_ros_msg(self) -> msg.Map:
        entities = list(map(lambda e: e.to_ros_msg(), self.entities))
        header = Header(stamp=self.timestamp)
        return msg.Map(header=header, entities=entities)
