import numpy as np

from . import ROSMessageConvertible


# Converts to/from an ROS point
class Vector2(ROSMessageConvertible):
    _matrix: np.array

    def x(self):
        self._matrix[0, 0]

    def y(self):
        self._matrix[1, 0]


class Transform2D:
    # Matrix with shape (1, 2)
    _matrix: np.array

    def __init__(self, matrix: np.array) -> None:
        assert matrix.shape == (
            3,
            3,
        ), "Transformation matrix must be a homogenous 3x3 matrix"
        self._matrix = matrix

    @staticmethod
    def new_rotation(angle: float) -> "Transform2D":
        c = np.cos(angle)
        s = np.sin(angle)

        matrix = np.array([[c, -s], [s, c]])
        # todo: homogenous matrix
        raise NotImplementedError
        Transform2D(matrix)

    @staticmethod
    def new_translation(v: Vector2) -> "Transform2D":
        raise NotImplementedError
