from mapping_common import shape
from mapping_common.transform import Transform2D, Vector2, Point2


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


def get_polygon():
    data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    p_list = []
    for i in range(0, 6, 2):
        p_list.append(Point2.new(data[i], data[i + 1]))

    return shape.Polygon(p_list)


def test_polygon_conversion():
    polygon = get_polygon()
    msg = polygon.to_ros_msg()
    polygon_conv = shape.Shape2D.from_ros_msg(msg)
    assert polygon == polygon_conv
