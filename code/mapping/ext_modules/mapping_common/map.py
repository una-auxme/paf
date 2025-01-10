from dataclasses import dataclass, field
from typing import List, Optional

from genpy.rostime import Time
from std_msgs.msg import Header
from mapping_common import entity
from mapping_common.entity import Entity, Car

from mapping import msg


@dataclass
class Map:
    """2 dimensional map for the intermediate layer

    General information:
    - 2D top-down map. The height(z) dimension is mostly useless
      for collision detection and path planning
    - The map is based on the local car position and is only built using sensor data.
      No global positioning via GPS or similar is used.
    - The map (0/0) is the center of the hero car
    - All transformations to entities are relative to
      the hero car's coordinate system (position/heading)
    - The map's x-axis is aligned with the heading of the hero car
    - The map's y-axis points to the left of the hero car
    - Coordinate system is a right-hand system like tf2 (can be visualized in RViz)
    - The map might include the hero car as the first entity in entities
    """

    timestamp: Time = Time()
    """The timestamp this map was created at.

    Should be the time when this map was initially sent off
    from the mapping_data_integration node.

    This timestamp is also the "freshest" compared to
    the timestamps of all entities included in the map
    """
    entities: List[Entity] = field(default_factory=list)
    """The entities this map consists out of

    Note that this list might also include the hero car (as first element of this list)
    """

    def hero(self) -> Optional[Entity]:
        """Returns the entity of the hero car if it is the first element of the map

        Returns:
            Optional[Entity]: Entity of the hero car
        """
        if len(self.entities) <= 0:
            return None
        hero = self.entities[0]
        if not hero.flags._is_hero:
            return None
        return hero

    def entities_without_hero(self) -> List[Entity]:
        """Returns the entities without the hero car

        Only checks if the first entity is_hero

        Returns:
            List[Entity]: Entities without the hero car
        """
        if self.hero() is not None:
            return self.entities[1:]
        return self.entities

    def get_entity_in_front(self) -> Optional[Entity]:
        """Returns the entity in front

        Rudimentary implementation without shapely
        Returns:
            Optional[Entity]: Entity in front
        """
        entities_in_front = []

        for e in self.entities_without_hero():

            translation = e.transform.translation()
            x = translation.x()
            y = translation.y()

            filter = entity.FlagFilter(is_collider=True)
            if y < 0.1 and y > -0.1 and x < -1.3 and e.matches_filter(filter):
                entities_in_front.append(e)

        if len(entities_in_front) > 0:
            return max(
                entities_in_front,
                key=lambda entity: entity.transform.translation().x(),
            )
        else:
            return None

    def get_entity_in_back(self) -> Optional[Entity]:
        """Returns the entity in front

        Rudimentary implementation without shapely
        Returns:
            Optional[Entity]: Entity in front
        """
        entities_in_back = []

        for e in self.entities_without_hero():

            translation = e.transform.translation()
            x = translation.x()
            y = translation.y()

            filter = entity.FlagFilter(is_collider=True)
            if y < 0.1 and y > -0.1 and x > 1.3 and e.matches_filter(filter):
                entities_in_back.append(e)

        if len(entities_in_back) > 0:
            return min(
                entities_in_back,
                key=lambda entity: entity.transform.translation().x(),
            )
        else:
            return None

    @staticmethod
    def from_ros_msg(m: msg.Map) -> "Map":
        entities = list(map(lambda e: Entity.from_ros_msg(e), m.entities))
        return Map(timestamp=m.header.stamp, entities=entities)

    def to_ros_msg(self) -> msg.Map:
        entities = list(map(lambda e: e.to_ros_msg(), self.entities))
        header = Header(stamp=self.timestamp)
        return msg.Map(header=header, entities=entities)
