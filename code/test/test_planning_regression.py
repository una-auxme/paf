"""Deterministic regression tests for planning helper behavior."""

from __future__ import annotations

import sys
from pathlib import Path
import importlib

import pytest

pytestmark = pytest.mark.unit

CODE_ROOT = Path(__file__).resolve().parents[1]
PLANNING_SRC = CODE_ROOT / "planning"
if str(PLANNING_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNING_SRC))


@pytest.fixture(scope="module")
def planning_help_functions():
    """Import planning helper module after path setup."""
    return importlib.import_module("planning.global_planner.help_functions")


def test_linear_interpolation_snapshot(planning_help_functions) -> None:
    """Check stable interpolation output for a known geometry."""
    points = planning_help_functions.linear_interpolation(
        (0.0, 0.0), (4.0, 0.0), interval_m=1.5
    )
    assert points == [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]


def test_scale_vector_zero_is_stable(planning_help_functions) -> None:
    """Check behavior for zero vectors remains deterministic and safe."""
    assert planning_help_functions.scale_vector((0.0, 0.0), 5.0) == (0, 0)


def test_position_stability_gate_accepts_after_bounded_unstable_window() -> None:
    """Prevent startup preplanning from waiting forever on drifting localization."""
    position_stability = importlib.import_module(
        "planning.global_planner.position_stability"
    )
    gate = position_stability.PositionStabilityGate(
        sample_count_target=3,
        stable_distance_m=0.5,
        max_unstable_samples=4,
    )

    decisions = [gate.update(float(x), 0.0) for x in range(7)]

    assert decisions == [False, False, False, False, False, False, True]


def test_global_planner_update_notifications_are_transient_local() -> None:
    """Late subscribers must still see route data availability notifications."""
    source = (
        CODE_ROOT / "planning/planning/global_planner/global_planner_node.py"
    ).read_text(encoding="utf-8")
    trajectory_publisher_block = source.split(
        "self.global_trajectory_updated_pub = self.create_publisher(",
        maxsplit=1,
    )[1].split("\n\n        self.speed_limit_updated_pub", maxsplit=1)[0]
    speed_limit_publisher_block = source.split(
        "self.speed_limit_updated_pub = self.create_publisher(",
        maxsplit=1,
    )[1].split("\n\n        # Service clients", maxsplit=1)[0]

    assert "DurabilityPolicy.TRANSIENT_LOCAL" in trajectory_publisher_block
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in speed_limit_publisher_block


def test_global_route_alignment_accepts_later_nearby_start() -> None:
    """Allow startup to recover when the first route pose is behind the ego pose."""
    route_alignment = importlib.import_module("planning.global_planner.route_alignment")

    route_points = [(0.0, 0.0), (100.0, 0.0), (205.0, 0.0), (300.0, 0.0)]

    assert (
        route_alignment.find_route_alignment_index(
            agent_position=(201.0, 3.0),
            route_points=route_points,
            max_distance_m=10.0,
        )
        == 2
    )


def test_global_route_alignment_rejects_route_without_nearby_pose() -> None:
    """Keep rejecting global routes that do not match the ego pose at all."""
    route_alignment = importlib.import_module("planning.global_planner.route_alignment")

    route_points = [(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]

    assert (
        route_alignment.find_route_alignment_index(
            agent_position=(350.0, 0.0),
            route_points=route_points,
            max_distance_m=10.0,
        )
        is None
    )


def test_planning_behaviors_use_lane_context_for_adjacent_lane_decisions() -> None:
    """Keep absent-lane checks distinct from blocked-lane checks in behaviors."""
    behavior_dir = CODE_ROOT / "planning/planning/behavior_agent/behaviors"

    lane_change_source = (behavior_dir / "lane_change.py").read_text(encoding="utf-8")
    overtake_source = (behavior_dir / "overtake.py").read_text(encoding="utf-8")
    parking_source = (behavior_dir / "leave_parking_space.py").read_text(
        encoding="utf-8"
    )

    assert lane_change_source.count("get_lane_context(") >= 2
    assert overtake_source.count("get_lane_context(") >= 3
    assert parking_source.count("get_lane_context(") >= 1
