import pytest

from mapping_common.map import Map

import test_entity

pytestmark = pytest.mark.unit


def test_map_conversion():
    m = Map()
    for i in range(200):
        c = test_entity.get_car()
        m.entities.append(c)
    msg = m.to_ros_msg()
    m_conv = Map.from_ros_msg(msg)
    assert m == m_conv
