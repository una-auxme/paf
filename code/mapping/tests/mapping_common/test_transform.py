from mapping_common.transform import Transform2D, Vector2, Point2


def test_translation():
    transl_v = Vector2.new(1.0, 2.0)
    transl = Transform2D.new_translation(transl_v)
    zero_p = Point2.zero()
    zero_v = Vector2.zero()

    assert transl * zero_p == Point2.from_vector(transl_v)
    assert transl * zero_v == Vector2.zero()


def test_translation2():
    transl_v = Vector2.new(1.0, 2.0)
    transl = Transform2D.new_translation(transl_v)

    assert transl.translation() == transl_v
