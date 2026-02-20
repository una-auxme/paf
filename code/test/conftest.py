"""Shared pytest fixtures for repository-wide tests."""

from __future__ import annotations

import random
from types import SimpleNamespace

import pytest


@pytest.fixture(autouse=True)
def deterministic_seed() -> None:
    """Set deterministic pseudo-random seeds for repeatable test behavior."""
    seed = 1337
    random.seed(seed)
    try:
        import numpy as np

        np.random.seed(seed)
    except ModuleNotFoundError:
        pass


@pytest.fixture
def fake_ros_clock() -> SimpleNamespace:
    """Provide a lightweight ROS-like clock object for pure-python tests."""
    return SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0))


@pytest.fixture
def snapshot_dir(tmp_path):
    """Return a per-test snapshot directory for golden-file style assertions."""
    snapshots_path = tmp_path / "snapshots"
    snapshots_path.mkdir(parents=True, exist_ok=True)
    return snapshots_path
