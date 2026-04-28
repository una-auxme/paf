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


def test_priority_cross_traffic_ignores_fast_entity_missing_conflict_area():
    hero = _make_car(0.0, 0.0, is_hero=True)
    offset_vehicle = _make_car(4.0, 24.0, 0.0, -7.5)
    map_obj = mapping_map.Map(entities=[hero, offset_vehicle])

    priority_clear, _ = intersection.check_priority_cross_traffic(
        map_obj, FakeTree([offset_vehicle])
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


def test_enter_pass_judge_line_triggers_once_braking_is_no_longer_comfortable():
    assert intersection._is_over_priority_pass_judge_line(4.0, 5.0)


def test_enter_pass_judge_line_allows_recheck_when_conflict_is_still_far():
    assert not intersection._is_over_priority_pass_judge_line(10.0, 2.0)


def test_priority_ttc_conflict_blocks_close_arrival_times():
    target_point = transform.Point2.new(15.0, 0.0)
    approaching = _make_car(12.0, 12.0, 0.0, -7.5)

    assert intersection._is_priority_ttc_conflict(approaching, target_point, 6.0)


def test_priority_conflict_point_tracks_actual_crossing_location():
    hero = _make_car(0.0, 0.0, is_hero=True)
    approaching = _make_car(12.0, 12.0, 0.0, -7.5)
    centerline = intersection._build_priority_check_centerline(hero)
    _, target_point = intersection._build_priority_check_mask(hero)

    conflict_point = intersection._get_priority_conflict_point(
        approaching,
        target_point,
        centerline,
        hero.get_width(),
    )

    assert conflict_point is not None
    assert conflict_point.x() == pytest.approx(12.0)
    assert conflict_point.y() == pytest.approx(0.0)


def test_priority_ttc_conflict_ignores_late_arrival_when_ego_is_committed():
    target_point = transform.Point2.new(15.0, 0.0)
    delayed = _make_car(24.0, 24.0, 0.0, -7.5)

    assert not intersection._is_priority_ttc_conflict(delayed, target_point, 10.0)


def test_priority_hysteresis_delays_new_blocking_decision():
    assert intersection._apply_priority_decision_hysteresis(False, True, 0.1)
    assert not intersection._apply_priority_decision_hysteresis(False, True, 0.3)


def test_priority_hysteresis_requires_stable_clear_before_release():
    assert not intersection._apply_priority_decision_hysteresis(True, False, 0.3)
    assert intersection._apply_priority_decision_hysteresis(True, False, 0.7)
