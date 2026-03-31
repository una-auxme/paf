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
