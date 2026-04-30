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
