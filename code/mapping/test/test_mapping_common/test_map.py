import pytest

from mapping_common import entity, shape, transform
from mapping_common.map import (
    LaneFreeState,
    LanePresence,
    Map,
)

pytestmark = pytest.mark.unit


def get_hero() -> entity.Car:
    return entity.Car(
        confidence=1.0,
        priority=1.0,
        shape=shape.Rectangle(4.5, 2.0),
        transform=transform.Transform2D.identity(),
        flags=entity.Flags(is_collider=True, is_hero=True),
    )


def get_lanemark(position_index: int, y: float, rotation: float = 0.0):
    return entity.Lanemarking(
        confidence=1.0,
        priority=1.0,
        shape=shape.Rectangle(30.0, 0.15),
        transform=transform.Transform2D.new_rotation_translation(
            rotation,
            transform.Vector2.new(0.0, y),
        ),
        flags=entity.Flags(is_lanemark=True),
        style=entity.Lanemarking.Style.SOLID,
        position_index=position_index,
        predicted=False,
    )


def get_blocking_car(x: float, y: float) -> entity.Car:
    return entity.Car(
        confidence=1.0,
        priority=1.0,
        shape=shape.Rectangle(4.5, 2.0),
        transform=transform.Transform2D.new_translation(transform.Vector2.new(x, y)),
        flags=entity.Flags(is_collider=True),
    )


def test_get_lane_presence_detects_present_absent_and_unknown():
    present_map = Map(
        entities=[
            get_hero(),
            get_lanemark(position_index=1, y=1.6),
            get_lanemark(position_index=2, y=4.6),
        ]
    )
    absent_map = Map(entities=[get_hero()])
    unknown_map = Map(
        entities=[
            get_hero(),
            get_lanemark(position_index=1, y=1.6, rotation=0.0),
            get_lanemark(position_index=2, y=4.6, rotation=0.4),
        ]
    )

    assert present_map.build_tree().get_lane_presence() is LanePresence.PRESENT
    assert absent_map.build_tree().get_lane_presence() is LanePresence.ABSENT
    assert unknown_map.build_tree().get_lane_presence() is LanePresence.UNKNOWN


def test_get_adjacent_lane_context_reports_presence_and_blocking():
    road_map = Map(
        entities=[
            get_hero(),
            get_lanemark(position_index=1, y=1.6),
            get_lanemark(position_index=2, y=4.6),
            get_blocking_car(x=6.0, y=3.1),
        ]
    )

    lane_context = road_map.build_tree(
        entity.FlagFilter(is_collider=True, is_hero=False)
    ).get_adjacent_lane_context(check_method="lanemarking")

    assert lane_context.has_left_lane() is True
    assert lane_context.left.lane_state is LaneFreeState.BLOCKED
    assert lane_context.left.marking_state is LaneFreeState.TO_BE_CHECKED
    assert lane_context.has_right_lane() is False
    assert lane_context.right.lane_state is LaneFreeState.MISSING_LANEMARK_ERR


def test_get_lane_context_preserves_absent_lane_information_with_fallback():
    road_map = Map(
        entities=[
            get_hero(),
            get_blocking_car(x=4.0, y=2.5),
        ]
    )

    lane_context = road_map.build_tree(
        entity.FlagFilter(is_collider=True, is_hero=False)
    ).get_lane_context(check_method="fallback")

    assert lane_context.has_lane() is False
    assert lane_context.presence is LanePresence.ABSENT
    assert lane_context.lane_state is LaneFreeState.BLOCKED
