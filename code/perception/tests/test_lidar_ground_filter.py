import numpy as np
import pytest

from perception.lidar_filter_utility import filter_ground_points

pytestmark = pytest.mark.unit


def make_points(rows):
    """Create a structured lidar point array with x, y, z, intensity fields."""
    return np.array(
        rows,
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
        ],
    )


def test_filter_ground_points_matches_static_height_window_without_pitch():
    points = make_points(
        [
            (1.0, 0.0, -1.5, 1),
            (1.0, 0.0, -1.0, 2),
            (1.0, 0.0, 1.6, 3),
        ]
    )

    filtered = filter_ground_points(points, z_min=-1.4, z_max=1.5, pitch_rad=0.0)

    assert filtered["intensity"].tolist() == [2]


def test_filter_ground_points_raises_lower_bound_with_positive_pitch():
    points = make_points(
        [
            (2.0, 0.0, -1.0, 1),
            (10.0, 0.0, -0.8, 2),
            (10.0, 0.0, 0.2, 3),
        ]
    )

    filtered = filter_ground_points(points, z_min=-1.4, z_max=1.5, pitch_rad=0.1)

    assert filtered["intensity"].tolist() == [1, 3]


def test_filter_ground_points_can_disable_pitch_compensation():
    points = make_points(
        [
            (10.0, 0.0, -0.8, 1),
            (10.0, 0.0, 0.2, 2),
        ]
    )

    filtered = filter_ground_points(
        points,
        z_min=-1.4,
        z_max=1.5,
        pitch_rad=0.1,
        enable_pitch_compensation=False,
    )

    assert filtered["intensity"].tolist() == [1, 2]
