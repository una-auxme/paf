from mapping_common import entity, shape, transform


def get_shape():
    return shape.Rectangle(1.0, 2.0)


def get_transform():
    return transform.Transform2D.new_rotation(1.0)


def get_car():
    return entity.Car(
        indicator=entity.Car.IndicatorState.LEFT,
        confidence=1.0,
        priority=1.0,
        shape=get_shape(),
        transform=get_transform(),
    )


def test_car_conversion():
    c = get_car()
    msg = c.to_ros_msg()
    c_conv = entity.Entity.from_ros_msg(msg)
    assert c == c_conv
