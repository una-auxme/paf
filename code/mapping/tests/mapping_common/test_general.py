from mapping_common.map import Map

import test_entity


def test_map_conversion():
    m = Map()
    for i in range(200):
        c = test_entity.get_car()
        m.entities.append(c)
    msg = m.to_ros_msg()
    m_conv = Map.from_ros_msg(msg)
    assert m == m_conv
