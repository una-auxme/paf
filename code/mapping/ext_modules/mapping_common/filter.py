from dataclasses import dataclass
from copy import deepcopy
from typing import List, Tuple, Optional, Callable
from uuid import UUID

import shapely

from .map import Map
from .entity import ShapelyEntity, Entity
from .shape import Polygon, Shape2D
from .transform import Transform2D


class MapFilter:
    """Abstract base class for all mapping filters"""

    def filter(self, map: Map) -> Map:
        raise NotImplementedError


@dataclass
class MergingFilter(MapFilter):
    # Grow merging:
    growth_distance: float
    min_merging_overlap: float

    def filter(self, map: Map) -> Map:
        tree = map.build_tree()
        query = tree.query_self(predicate="dwithin", distance=self.growth_distance)

        # Entities that will be removed out of the tree: Key: UUID,
        # Value: UUID of the entity it was merged with
        #   This UUID will be part of the modified_entities
        removed_entities = dict()
        # Entities that are modified: Key: UUID, Value: Entity
        modified_entities = dict()

        for pair in query:
            merge_result = try_merge_pair(
                pair,
                lambda p: grow_merge_pair(
                    p, self.growth_distance, self.min_merging_overlap
                ),
            )
            if merge_result is None:
                continue
            (modified, removed_uuid) = merge_result
            removed_entities[removed_uuid] = modified.uuid
            modified_entities[modified.uuid] = modified

        merged_entities: List[Entity] = []
        for entity in map.entities:
            if entity.uuid in removed_entities:
                continue
            if entity.uuid in modified_entities:
                new_entry = modified_entities[entity.uuid]
            else:
                new_entry = entity
            merged_entities.append(new_entry)

        merged_map = Map(timestamp=map.timestamp, entities=merged_entities)
        return merged_map


def try_merge_pair(
    pair: Tuple[ShapelyEntity, ShapelyEntity],
    shape_merge_fn: Callable[
        [Tuple[ShapelyEntity, ShapelyEntity]], Optional[Tuple[Transform2D, Shape2D]]
    ],
) -> Optional[Tuple[Entity, UUID]]:
    # todo: Check if entity is even mergeable

    merged_shape = shape_merge_fn(pair)
    if merged_shape is None:
        return None
    (transform, shape) = merged_shape

    if pair[0].entity.priority >= pair[1].entity.priority:
        base_entity_idx = 0
        merge_entity_idx = 1
    else:
        base_entity_idx = 1
        merge_entity_idx = 0
    base_entity: Entity = pair[base_entity_idx].entity
    merge_entity: Entity = pair[merge_entity_idx].entity

    modified = deepcopy(base_entity)
    modified.transform = transform
    modified.shape = shape

    if modified.motion is None:
        modified.motion = merge_entity.motion
    if modified.tracking_info is None:
        modified.tracking_info = merge_entity.tracking_info

    return (modified, merge_entity.uuid)


def grow_merge_pair(
    pair: Tuple[ShapelyEntity, ShapelyEntity],
    growth_distance: float,
    min_merging_overlap: float,
) -> Optional[Tuple[Transform2D, Shape2D]]:
    growns = [_grow_polygon(e.poly, growth_distance) for e in pair]
    areas = [e.area for e in growns]

    intersection = shapely.intersection(growns[0], growns[1])
    if intersection.is_empty:
        return None
    if not isinstance(intersection, shapely.Polygon):
        return None
    i_area = intersection.area
    area_overlaps = [i_area / a for a in areas]
    # the bigger the area_fract, the more of a shape lies
    # within the intersecting area.
    if max(area_overlaps) < min_merging_overlap:
        return None

    grown_union = shapely.union(growns[0], growns[1])
    if not isinstance(grown_union, shapely.Polygon):
        return None
    shrunk_union = _grow_polygon(grown_union, -growth_distance)
    if len(shrunk_union.exterior.coords) < 3:
        return None
    shape = Polygon.from_shapely(shrunk_union, make_centered=True)
    transform = shape.offset
    shape.offset = Transform2D.identity()
    return (transform, shape)


def _grow_polygon(p: shapely.Polygon, distance: float) -> shapely.Polygon:
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
    return shapely.Polygon(grown.coords)
