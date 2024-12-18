from mapping_common import shape


def test_rectangle_conversion():
    s = shape.Rectangle(1.0, 2.0)
    msg = s.to_ros_msg()
    s_conv = shape.Shape2D.from_ros_msg(msg)
    assert s == s_conv
