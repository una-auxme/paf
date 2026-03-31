import math
from types import SimpleNamespace

import pytest

from mapping_common import entity, map as mapping_map, shape, transform
from planning.behavior_agent.behaviors import intersection

pytestmark = pytest.mark.unit


class FakeTree:
    def __init__(self, entities):
        self.entities = entities

    def get_overlapping_entities(self, mask):
        del mask
        return [SimpleNamespace(entity=e) for e in self.entities]


def _make_car(
    x: float,
    y: float,
    vx: float = 0.0,
    vy: float = 0.0,
    *,
    is_hero: bool = False,
) -> entity.Car:
    flags = entity.Flags(is_collider=not is_hero, is_hero=is_hero)
    motion = entity.Motion2D(linear_motion=transform.Vector2.new(vx, vy))
    return entity.Car(
        confidence=1.0,
        priority=1.0,
        shape=shape.Rectangle(4.0, 2.0),
        transform=transform.Transform2D.new_rotation_translation(
            0.0, transform.Vector2.new(x, y)
        ),
        motion=motion,
        flags=flags,
    )


def test_priority_cross_traffic_blocks_fast_approaching_entity():
    hero = _make_car(0.0, 0.0, is_hero=True)
    approaching = _make_car(12.0, 12.0, 0.0, -7.5)
    map_obj = mapping_map.Map(entities=[hero, approaching])

    priority_clear, _ = intersection.check_priority_cross_traffic(
        map_obj, FakeTree([approaching])
    )

    assert not priority_clear


def test_priority_cross_traffic_ignores_fast_entity_moving_away():
    hero = _make_car(0.0, 0.0, is_hero=True)
    moving_away = _make_car(12.0, 12.0, 0.0, 7.5)
    map_obj = mapping_map.Map(entities=[hero, moving_away])

    priority_clear, _ = intersection.check_priority_cross_traffic(
        map_obj, FakeTree([moving_away])
    )

    assert priority_clear


def test_priority_check_mask_stays_world_aligned_while_turning():
    hero = _make_car(0.0, 0.0, is_hero=True)
    reference_pose = transform.Transform2D.identity()
    current_pose = transform.Transform2D.new_rotation_translation(
        math.pi / 2.0, transform.Vector2.zero()
    )

    _, target_point = intersection._build_priority_check_mask(
        hero,
        reference_pose=reference_pose,
        current_pose=current_pose,
    )

    expected_distance = intersection.PRIORITY_CHECK_DISTANCE + hero.get_front_x()
    assert target_point.x() == pytest.approx(0.0)
    assert target_point.y() == pytest.approx(-expected_distance)
