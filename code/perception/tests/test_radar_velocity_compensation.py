import numpy as np
import pytest

from mapping_common.transform import Vector2
from perception.radar_node import (
    _compensate_radar_radial_velocity,
    _sensor_ego_velocity,
)

pytestmark = pytest.mark.unit


def test_sensor_ego_velocity_includes_sensor_offset_rotation():
    sensor_velocity = _sensor_ego_velocity(
        Vector2.new(2.0, 1.5),
        yaw_rate=1.0,
    )

    assert sensor_velocity.x() == pytest.approx(-1.5)
    assert sensor_velocity.y() == pytest.approx(2.0)


def test_radar_velocity_compensation_cancels_stationary_translation():
    ego_speed = 8.0
    azimuth = np.pi / 3.0
    radial_velocity = -(ego_speed * np.cos(azimuth))

    compensated = _compensate_radar_radial_velocity(
        radial_velocity=radial_velocity,
        azimuth=azimuth,
        ego_speed=ego_speed,
    )

    assert np.allclose([compensated.x(), compensated.y()], [0.0, 0.0], atol=1e-6)


@pytest.mark.parametrize("azimuth", [0.0, np.pi / 4.0, np.pi / 2.0])
def test_radar_velocity_compensation_cancels_stationary_yaw_motion(azimuth):
    sensor_position = Vector2.new(2.0, 1.5)
    sensor_velocity = _sensor_ego_velocity(sensor_position, yaw_rate=1.0)
    radial_velocity = -(
        sensor_velocity.x() * np.cos(azimuth) + sensor_velocity.y() * np.sin(azimuth)
    )

    compensated = _compensate_radar_radial_velocity(
        radial_velocity=radial_velocity,
        azimuth=azimuth,
        yaw_rate=1.0,
        sensor_position=sensor_position,
    )

    assert np.allclose([compensated.x(), compensated.y()], [0.0, 0.0], atol=1e-6)
