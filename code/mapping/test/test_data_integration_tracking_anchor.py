import numpy as np
import pytest

from mapping.data_integration import (
    _build_polygon_shape_and_transform,
    _select_tracking_points,
)

pytestmark = pytest.mark.unit


def test_select_tracking_points_prefers_fresh_lidar_points():
    cluster_points_xy = np.array(
        [[0.0, 0.0], [0.0, 2.0], [4.0, 0.0], [4.0, 2.0]],
        dtype=float,
    )
    is_buffered_mask = np.array([True, True, False, False], dtype=bool)

    tracking_points_xy = _select_tracking_points(cluster_points_xy, is_buffered_mask)

    assert np.array_equal(tracking_points_xy, cluster_points_xy[2:])


def test_build_polygon_shape_and_transform_keeps_full_hull_with_fresh_anchor():
    cluster_points_xy = np.array(
        [[0.0, 0.0], [0.0, 2.0], [4.0, 2.0], [4.0, 0.0]],
        dtype=float,
    )
    fresh_points_xy = np.array([[4.0, 0.0], [4.0, 2.0]], dtype=float)

    polygon = _build_polygon_shape_and_transform(cluster_points_xy, fresh_points_xy)

    assert polygon is not None
    shape, transform = polygon

    translation = transform.translation()
    assert translation.x() == pytest.approx(4.0)
    assert translation.y() == pytest.approx(1.0)

    polygon_xy = shape.to_shapely(transform)
    assert np.allclose(polygon_xy.bounds, (0.0, 0.0, 4.0, 2.0))

    local_x = [point.x() for point in shape.points]
    assert min(local_x) == pytest.approx(-4.0)
    assert max(local_x) == pytest.approx(0.0)
