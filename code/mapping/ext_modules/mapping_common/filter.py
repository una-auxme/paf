from dataclasses import dataclass
from copy import deepcopy
from typing import List, Tuple, Optional, Callable
from uuid import UUID

import shapely
import rospy

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
        # Value: Entity it was merged with
        removed_entities = dict()
        # Entities that are modified: Key: UUID, Value: Entity
        modified_entities = dict()

        while len(query) > 0:
            pair = query.pop()
            if pair[0].entity.uuid == pair[1].entity.uuid and pair[0].poly.equals(
                pair[1].poly
            ):
                rospy.logerr(
                    "MergingFilter: query contains a pair of the same \
                        (uuid+shape) entity. Skipping pair."
                )
                continue
            merge_result = try_merge_pair(
                pair,
                lambda p: grow_merge_pair(
                    p, self.growth_distance, self.min_merging_overlap
                ),
            )
            if merge_result is None:
                continue
            (modified, removed_uuid) = merge_result
            # We need to check if stuff we are merging was already
            # affected by another merge
            if removed_uuid in modified_entities:
                # We want to remove an entity that has already been
                #   modified by another merge
                # -> Do not remove it and try to merge our modified result
                #   with the other
                query.append(
                    (
                        modified.to_shapely(),
                        modified_entities.pop(removed_uuid).to_shapely(),
                    )
                )
            elif modified.uuid != removed_uuid:
                removed_entities[removed_uuid] = modified
            if modified.uuid in removed_entities:
                # The entity we want to use as base was already part of a merge
                #   and was merged into another entity
                # -> Unremove it and queue another merge with
                #   the entity it was merged into
                query.append(
                    (
                        modified.to_shapely(),
                        removed_entities.pop(modified.uuid).to_shapely(),
                    )
                )
            if modified.uuid in modified_entities:
                # The entity was already part of another merge
                # -> Queue another merge between those two
                # Note that this leads to duplicate uuids while the algorithm runs
                query.append(
                    (
                        modified.to_shapely(),
                        modified_entities.pop(modified.uuid).to_shapely(),
                    )
                )
            else:
                modified_entities[modified.uuid] = modified

        # remove any modified_entities that are part of removed_entities
        for uuid in removed_entities.keys():
            if uuid in modified_entities:
                del modified_entities[uuid]
        merged_entities: List[Entity] = []
        # append the entities of the original map
        for entity in map.entities:
            if entity.uuid in removed_entities:
                continue
            if entity.uuid in modified_entities:
                new_entry = modified_entities.pop(entity.uuid)
            else:
                new_entry = entity
            merged_entities.append(new_entry)
        # # append any modified entities that do not have uuids of the original map
        # for entry in modified_entities.values():
        #     merged_entities.append(entry)

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
    growns_maybe_none = [_grow_polygon(e.poly, growth_distance) for e in pair]
    for g in growns_maybe_none:
        if g is None:
            return None
    growns: List[shapely.Polygon] = growns_maybe_none

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
    # Entities are only merged, if one of them at least
    # overlaps the other by min_merging_overlap.
    if max(area_overlaps) < min_merging_overlap:
        return None

    grown_union = shapely.union(growns[0], growns[1])
    if not isinstance(grown_union, shapely.Polygon):
        return None
    shrunk_union = _grow_polygon(grown_union, -growth_distance)
    if shrunk_union is None:
        return None
    shape = Polygon.from_shapely(shrunk_union, make_centered=True)
    transform = shape.offset
    shape.offset = Transform2D.identity()
    return (transform, shape)


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
