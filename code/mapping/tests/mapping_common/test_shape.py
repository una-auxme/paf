from mapping_common import shape
from mapping_common.transform import Transform2D, Vector2


def test_rectangle_conversion():
    s = shape.Rectangle(1.0, 2.0)
    msg = s.to_ros_msg()
    s_conv = shape.Shape2D.from_ros_msg(msg)
    assert s == s_conv


def test_circle_conversion():
    offset = Transform2D.new_translation(Vector2.new(1.0, 0.0))
    s = shape.Circle(3.0, offset)
    msg = s.to_ros_msg()
    s_conv = shape.Shape2D.from_ros_msg(msg)
    assert s == s_conv
