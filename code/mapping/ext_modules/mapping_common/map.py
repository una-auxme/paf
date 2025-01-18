from dataclasses import dataclass, field
from typing import List, Optional, Callable, Literal, Tuple

import shapely
from shapely import STRtree
import numpy as np
import numpy.typing as npt

from genpy.rostime import Time
from std_msgs.msg import Header

from mapping_common.entity import Entity, FlagFilter, ShapelyEntity

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

    def build_tree(
        self,
        f: Optional[FlagFilter] = None,
        filter_fn: Optional[Callable[[Entity], bool]] = None,
    ) -> "MapTree":
        """Creates a filtered MapTree

        **IMPORTANT**: The map
        MUST NOT BE MODIFIED WHILE USING THE TREE,
        otherwise results will be invalid or crash

         Useful for for quickly calculating which entities of a map are
        the nearest or (intersect, touch, etc.) with a given geometry

        Args:
            f (Optional[FlagFilter], optional): Filtering with FlagFilter.
                Defaults to None.
            filter_fn (Optional[Callable[[Entity], bool]], optional):
                Filtering with function/lambda. Defaults to None.

        Returns:
            MapTree
        """
        return MapTree(self, f, filter_fn)

    def filtered(
        self,
        f: Optional[FlagFilter] = None,
        filter_fn: Optional[Callable[[Entity], bool]] = None,
    ) -> List[Entity]:
        """Filters self.entities

        Args:
            f (Optional[FlagFilter], optional): Filtering with FlagFilter.
                Defaults to None.
            filter_fn (Optional[Callable[[Entity], bool]], optional):
                Filtering with function/lambda. Defaults to None.

        Returns:
            List[Entity]: List of entities both filters were matching for
        """
        return [e for e in self.entities if _entity_matches_filter(e, f, filter_fn)]

    @staticmethod
    def from_ros_msg(m: msg.Map) -> "Map":
        entities = list(map(lambda e: Entity.from_ros_msg(e), m.entities))
        return Map(timestamp=m.header.stamp, entities=entities)

    def to_ros_msg(self) -> msg.Map:
        entities = list(map(lambda e: e.to_ros_msg(), self.entities))
        header = Header(stamp=self.timestamp)
        return msg.Map(header=header, entities=entities)


@dataclass(init=False)
class MapTree:
    """An acceleration structure around the shapely.STRtree

    **IMPORTANT**: The map this tree was created with
    MUST NOT BE MODIFIED WHILE USING THE TREE,
    otherwise results will be invalid or crash

    Useful for for quickly calculating which entities of a map are
    the nearest or (intersect, touch, etc.) with a given geometry

    The map's entities can optionally be filtered upon tree creation
    """

    _str_tree: STRtree
    filtered_entities: List[ShapelyEntity]
    """Only the entities of this tree that weren't filtered out from the map.

    Also includes their shapely.Polygon
    """
    map: Map
    """The map this tree was created with
    """

    def __init__(
        self,
        map: Map,
        f: Optional[FlagFilter] = None,
        filter_fn: Optional[Callable[[Entity], bool]] = None,
    ):
        """Creates a shapely.STRtree based on the given map and filtering

        Both filtering methods can be combined.
        Both filters need to match for the entity to match.

        Args:
            map (Map)
            f (Optional[FlagFilter], optional): Filtering with FlagFilter.
                Defaults to None.
            filter_fn (Optional[Callable[[Entity], bool]], optional):
                Filtering with function/lambda. Defaults to None.
        """
        self.map = map
        self.filtered_entities = [e.to_shapely() for e in map.filtered(f, filter_fn)]
        self._str_tree = STRtree(geoms=[e.poly for e in self.filtered_entities])

    def _idxs_to_entity(self, idxs: npt.NDArray) -> List[ShapelyEntity]:
        return [self.filtered_entities[i] for i in idxs]

    def nearest(self, geo: shapely.Geometry) -> Optional[ShapelyEntity]:
        """Returns the nearest Entity inside the tree based on geo

        More information here:
        https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.nearest

        Args:
            geo (shapely.Geometry): Geometry to calculate the nearest entity to

        Returns:
            Optional[ShapelyEntity]: If the tree is empty, will return None.
                Otherwise will return the nearest Entity
        """
        idx = self._str_tree.nearest(geo)
        if idx is None:
            return None
        return self.filtered_entities[idx]

    def query(
        self,
        geo: shapely.Geometry,
        predicate: Optional[
            Literal[
                "intersects",
                "within",
                "contains",
                "overlaps",
                "crosses",
                "touches",
                "covers",
                "covered_by",
                "contains_properly",
                "dwithin",
            ]
        ] = None,
        distance: Optional[float] = None,
    ) -> List[ShapelyEntity]:
        """Calculates which entities interact with *geo*

        More information here:
        https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query

        Args:
            geo (shapely.Geometry): The geometry to query with
            predicate (Optional[ Literal[ &quot;intersects&quot;, &quot;within&quot;,
            &quot;contains&quot;, &quot;overlaps&quot;, &quot;crosses&quot;,
            &quot;touches&quot;, &quot;covers&quot;, &quot;covered_by&quot;,
            &quot;contains_properly&quot;, &quot;dwithin&quot;, ] ], optional):
            Which interaction to filter for. Defaults to None.
            distance (Optional[float], optional):
            Must only be set for the &quot;dwithin&quot; predicate
            and controls its distance. Defaults to None.

        Returns:
            List[ShapelyEntity]: The List of queried entities in the tree
        """
        idxs: npt.NDArray[np.int64] = self._str_tree.query(
            geo, predicate=predicate, distance=distance
        )
        return self._idxs_to_entity(idxs)

    def query_nearest(
        self,
        geo: shapely.Geometry,
        max_distance: Optional[float] = None,
        exclusive: bool = False,
        all_matches: bool = True,
    ) -> List[Tuple[ShapelyEntity, float]]:
        """Queries the distance from *geo* to its nearest entities in the tree

        More information here:
        https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query_nearest

        Args:
            geo (shapely.Geometry): The geometry to query with
            max_distance (Optional[float], optional): Maximum distance for the query.
                Defaults to None.
            exclusive (bool, optional): If True, ignores entities
            with a shape equal to geo. Defaults to False.
            all_matches (bool, optional): If True, all equidistant and intersected
            geometries will be returned. If False only the nearest. Defaults to True.

        Returns:
            List[Tuple[ShapelyEntity, float]]: A List of Tuples.
            Each contains a queried entity and its distance to geo
        """
        query: Tuple[npt.NDArray[np.int64], npt.NDArray[np.float64]] = (
            self._str_tree.query_nearest(
                geo,
                max_distance=max_distance,
                return_distance=True,
                exclusive=exclusive,
                all_matches=all_matches,
            )
        )
        result: List[Tuple[ShapelyEntity, float]] = []
        for idx, distance in zip(query[0], query[1]):
            result.append((self.filtered_entities[idx], distance))
        return result


def _entity_matches_filter(
    e: Entity,
    f: Optional[FlagFilter] = None,
    filter_fn: Optional[Callable[[Entity], bool]] = None,
) -> bool:
    if f is not None:
        if not e.matches_filter(f):
            return False
    if filter_fn is not None:
        if not filter_fn(e):
            return False
    return True
