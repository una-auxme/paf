"""Contains filter-related functions

**[API documentation](/doc/mapping/generated/mapping_common/filter.md)**
"""

from dataclasses import dataclass
from copy import deepcopy
from typing import List, Tuple, Optional, Callable
from uuid import UUID

import rospy
import shapely

from .map import Map
from .entity import ShapelyEntity, Entity, FlagFilter, Pedestrian
from .shape import Polygon, Shape2D
from .transform import Transform2D


class MapFilter:
    """Abstract base class for all mapping filters"""

    def filter(self, map: Map) -> Map:
        """Filters the map.

        Look into the class description for what the filter does

        Args:
            map (Map): Map to filter

        Returns:
            Map: New map with filter applied.
                Note that unmodified entities are NOT deepcopied.
        """
        raise NotImplementedError


@dataclass
class LaneIndexFilter(MapFilter):
    """Updates the Index of lanemark Entities if duplicates have been removed.

    !!!Must be called after GrowthMergingFilter!!!

    - Calculates the y coordinates of the intersection with y axis of each lanemarking.
    - Gives position_index according to y position:
        - 1 = lane next to the car on the left.
        - 2 = second lanemark on the left.
        - -1 = lane next to the car on the right.
        - etc.

    Then returns the updated map with all Entities
    """

    def filter(self, map) -> Map:
        try:
            lanemark_f = FlagFilter(is_lanemark=True)
            other_f = FlagFilter(is_lanemark=False)
            lanemarkings = map.filtered(lanemark_f)
            other_entities = map.filtered(other_f)

            intersections = map.get_lane_y_axis_intersections(direction="both")
            y_values = [(uuid, intersections[uuid][1]) for uuid in intersections]
            # separate negative and positive values
            positive_y = sorted(
                [(uuid, y) for uuid, y in y_values if y > 0], key=lambda x: x[1]
            )
            negative_y = sorted(
                [(uuid, y) for uuid, y in y_values if y < 0], key=lambda x: abs(x[1])
            )
            # creates a dictionary for the labels with uuid as keys
            labels = {}
            for i, (uuid, _) in enumerate(positive_y):
                labels[uuid] = i + 1  # starts at 1
            for i, (uuid, _) in enumerate(negative_y):
                labels[uuid] = -(i + 1)  # starts at
            # iterate through all Lanemark Entities and give it new position index
            for lanemarking in lanemarkings:
                lanemarking.position_index = labels.get(lanemarking.uuid, 0)

            # insert all updated Lanemarks and non_lanemarking Entities
            updated_map = Map(map.timestamp, other_entities + lanemarkings)

            return updated_map
        except Exception as e:
            rospy.logwarn(f"Error in LaneIndexFilter: {e}")
            return map  # Return original map on error


@dataclass
class GrowPedestriansFilter(MapFilter):
    """Grow Pedestrians by 0.5 meter for a better detection of them (e.g. for the ACC)

    !!!Must be called after GrowthMergingFilter!!!

    - Iterates over all entities
    - If entity is a Pedestrian: Grow them by 0.5 meter

    Then returns the updated map with all Entities
    """

    def filter(self, map) -> Map:
        for entity in map.entities:
            if isinstance(entity, Pedestrian):
                shape_grown = _grow_polygon(entity.shape.to_shapely(), 0.5)
                if shape_grown is not None:
                    entity.shape = Polygon.from_shapely(shape_grown)
        return map


@dataclass
class GrowthMergingFilter(MapFilter):
    """Merges entities in the map with growing them

    Basic (very simplified) function:
    - Compares pairs of entities that are in the vicinity of each other
    - For each pair:
        - Checks if they are mergeable at all
        - Grows their shape based on growth_distance
            -> This is done in order to also catch small noisy entities
            (outside) around a bigger entity
        - Checks the min_merging_overlap_percent and min_merging_overlap_area
            based on the intersection of the grown shapes
        - -> Merge if at least one of them is true
        - Creates a grown union for the merged entity
        - Shrinks the union by growth_distance
    - It then deletes all entities that got merged into another
        and returns the resulting map
    """

    growth_distance: float
    # Both checks ar OR-ed for merging
    min_merging_overlap_percent: float
    """Min overlap of the grown shapes in percent
    """
    min_merging_overlap_area: float
    """Min overlap of the grown shapes in m2
    """
    simplify_tolerance: float

    def filter(self, map: Map) -> Map:
        tree = map.build_tree()
        query = tree.query_self(predicate="dwithin", distance=self.growth_distance)

        # Entities that will be removed out of the tree: Key: UUID,
        # Value: ShapelyEntity it was merged with
        removed_entities = dict()
        # Entities that are modified: Key: UUID, Value: ShapelyEntity
        modified_entities = dict()

        while len(query) > 0:
            pair = query.pop()
            uuid0 = pair[0].entity.uuid
            uuid1 = pair[1].entity.uuid
            if uuid0 == uuid1:
                continue

            if uuid0 in removed_entities and uuid1 in removed_entities:
                continue
            if uuid0 in removed_entities:
                query.append((removed_entities[uuid0], pair[1]))
                continue
            if uuid1 in removed_entities:
                query.append((pair[0], removed_entities[uuid1]))
                continue

            if uuid0 in modified_entities and uuid1 in modified_entities:
                pair = (
                    modified_entities[uuid0],
                    modified_entities[uuid1],
                )
            elif uuid0 in modified_entities:
                pair = (modified_entities[uuid0], pair[1])
            elif uuid1 in modified_entities:
                pair = (pair[0], modified_entities[uuid1])

            merge_result = _try_merge_pair(
                pair,
                lambda p: _grow_merge_pair(
                    p,
                    self.growth_distance,
                    self.min_merging_overlap_percent,
                    self.min_merging_overlap_area,
                    self.simplify_tolerance,
                ),
            )
            if merge_result is None:
                continue
            (modified, removed_uuid) = merge_result
            # print(f"Merged {modified.uuid} and {removed_uuid}")
            if modified.entity.uuid != removed_uuid:
                removed_entities[removed_uuid] = modified
            modified_entities[modified.entity.uuid] = modified

        merged_entities: List[Entity] = []
        # append the entities of the original map based on modifications and removals
        for entity in map.entities:
            if entity.uuid in removed_entities:
                continue
            if entity.uuid in modified_entities:
                new_entry: Entity = modified_entities.pop(entity.uuid).entity
            else:
                new_entry: Entity = entity
            merged_entities.append(new_entry)

        merged_map = Map(timestamp=map.timestamp, entities=merged_entities)
        return merged_map


def _try_merge_pair(
    pair: Tuple[ShapelyEntity, ShapelyEntity],
    shape_merge_fn: Callable[
        [Tuple[ShapelyEntity, ShapelyEntity]],
        Optional[Tuple[Transform2D, Shape2D, shapely.Polygon]],
    ],
) -> Optional[Tuple[ShapelyEntity, UUID]]:
    """Tries to merge an entity-pair

    Args:
        pair (Tuple[ShapelyEntity, ShapelyEntity]): Pair of entities to merge
        shape_merge_fn (Callable[ [Tuple[ShapelyEntity, ShapelyEntity]],
            Optional[Tuple[Transform2D, Shape2D, shapely.Polygon]] ]):
                Dedicated merging function that merges the pair based on their shape
                and then returns the transform and shape of the resulting merged entity.

    Returns:
        Optional[Tuple[Entity, UUID]]:
            If no merging happened for whatever reason, returns None.
            Otherwise the tuple contains the resulting merged entity
            and the uuid of the entity that was merged into the other
                (-> should be deleted in the map)
    """
    if not pair[0].entity.is_mergeable_with(pair[1].entity):
        return None

    merged_shape = shape_merge_fn(pair)
    if merged_shape is None:
        return None
    (transform, shape, merged_poly) = merged_shape

    if pair[0].entity.priority >= pair[1].entity.priority:
        base_entity_idx = 0
        merge_entity_idx = 1
    else:
        base_entity_idx = 1
        merge_entity_idx = 0

    base_entity = pair[base_entity_idx].entity
    merge_entity = pair[merge_entity_idx].entity

    modified = deepcopy(base_entity)

    modified.transform = transform
    modified.shape = shape

    # Now we adjust/copy over any other attributes than shape and transform
    if modified.motion is None:
        modified.motion = merge_entity.motion
    if modified.tracking_info is None:
        modified.tracking_info = merge_entity.tracking_info
    modified.sensor_id += merge_entity.sensor_id
    confidences: List[float] = [modified.confidence, merge_entity.confidence]
    min_conf: float = min(confidences)
    max_conf: float = max(confidences)
    modified.confidence = min(1.0, max_conf + (min_conf * (1.0 - max_conf)))

    return (ShapelyEntity(entity=modified, poly=merged_poly), merge_entity.uuid)


def _grow_merge_pair(
    pair: Tuple[ShapelyEntity, ShapelyEntity],
    growth_distance: float,
    min_merging_overlap_percent: float,
    min_merging_overlap_area: float,
    simplify_tolerance: float,
) -> Optional[Tuple[Transform2D, Shape2D, shapely.Polygon]]:
    """Merges a pair of entities based on their shape

    Basic (very simplified) function:
    - Grows their shape based on growth_distance
    - Checks the min_merging_overlap_percent and min_merging_overlap_area
        based on the intersection of the grown shapes
    - -> Merge if at least one of them is true
    - Creates a grown union for the merged entity
    - Shrinks the union by growth_distance

    Args:
        pair (Tuple[ShapelyEntity, ShapelyEntity]): Pair of entities to merge
        growth_distance (float)
        min_merging_overlap_percent (float): Min overlap of the grown shapes in percent
        min_merging_overlap_area (float): Min overlap of the grown shapes in m2

    Returns:
        Optional[Tuple[Transform2D, Shape2D, shapely.Polygon]]:
            If no merging happened for whatever reason, returns None.
            Otherwise returns the transform and shape of the resulting merged entity
    """
    # The hero car must not grow.
    # Otherwise an entity we crash into might be merged into the hero
    # Growing lanemarks or stopmarks also makes little sense
    grow_whitefilter = FlagFilter(is_hero=False, is_lanemark=False, is_stopmark=False)
    needs_shrinking = False
    if pair[0].entity.matches_filter(grow_whitefilter) and pair[
        1
    ].entity.matches_filter(grow_whitefilter):
        growns_maybe_none = [_grow_polygon(e.poly, growth_distance) for e in pair]
        for g in growns_maybe_none:
            if g is None:
                return None
        needs_shrinking = True
    else:
        growns_maybe_none = [pair[0].poly, pair[1].poly]

    growns: List[shapely.Polygon] = growns_maybe_none

    # Intersection of both grown shapes. Used for area check
    grown_intersection = shapely.intersection(growns[0], growns[1])
    if grown_intersection.is_empty:
        return None
    if not isinstance(grown_intersection, shapely.Polygon):
        return None
    # Intersection of one grown and one original shape. Used for percentage check
    g_o_intersections = [
        shapely.intersection(growns[0], pair[1].poly),
        shapely.intersection(growns[1], pair[0].poly),
    ]
    percentage_overlaps = [
        g_o_intersections[0].area / pair[1].poly.area,
        g_o_intersections[1].area / pair[0].poly.area,
    ]
    # the bigger the percentage_overlaps, the more of a shape lies
    # within the intersecting area.
    # Entities are only merged, if one of them at least
    # overlaps the other by min_merging_overlap.
    if (
        max(percentage_overlaps) < min_merging_overlap_percent
        and grown_intersection.area < min_merging_overlap_area
    ):
        return None

    merged_union = shapely.union(growns[0], growns[1])
    if not isinstance(merged_union, shapely.Polygon):
        return None
    if needs_shrinking:
        merged_union = _grow_polygon(merged_union, -growth_distance)
        if merged_union is None:
            return None
    merged_union = merged_union.simplify(simplify_tolerance, preserve_topology=True)
    shape = Polygon.from_shapely(merged_union, make_centered=True)
    transform = shape.offset
    shape.offset = Transform2D.identity()
    return (transform, shape, merged_union)


def _grow_polygon(p: shapely.Polygon, distance: float) -> Optional[shapely.Polygon]:
    """Grows or shrinks a polygon by a distance

    Args:
        p (shapely.Polygon)
        distance (float): Positive: grow, Negative: shrink

    Returns:
        shapely.Polygon
    """
    exterior = p.exterior
    # A positive distance means the curve is offset to the left
    # This means we need to invert the distance if our outer ring is counter-clockwise
    if shapely.is_ccw(exterior):
        distance = -distance
    grown = exterior.offset_curve(distance=distance)
    if len(grown.coords) < 3:
        return None
    poly = shapely.Polygon(grown.coords)
    if not poly.is_valid:
        return None
    return poly
