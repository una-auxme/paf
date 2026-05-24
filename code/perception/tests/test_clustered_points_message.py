import numpy as np
import pytest

from perception.perception_utils import array_to_clustered_points
from rclpy.time import Time

pytestmark = pytest.mark.unit


def test_array_to_clustered_points_serializes_buffered_mask():
    points = np.array([[1.0, 2.0, 0.5], [2.5, -1.0, 0.25]])
    point_indices = np.array([3, 3])
    is_buffered_array = np.array([False, True])

    clustered_points = array_to_clustered_points(
        Time(seconds=12),
        points,
        point_indices,
        is_buffered_array=is_buffered_array,
        header_id="hero/LIDAR",
    )

    assert clustered_points.header.frame_id == "hero/LIDAR"
    assert list(clustered_points.index_array) == [3, 3]
    assert list(clustered_points.is_buffered_array) == [False, True]
    assert list(clustered_points.cluster_points_array) == pytest.approx(
        points.flatten()
    )
