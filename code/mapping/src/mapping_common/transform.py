from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from geometry_msgs import msg as geometry_msgs
from mapping import msg


@dataclass
class Point2:
    # Matrix with shape (2)
    _matrix: NDArray


@dataclass
class Vector2:
    # Matrix with shape (2)
    _matrix: NDArray

    @staticmethod
    def new(x: float, y: float) -> "Vector2":
        m = np.array([x, y], dtype=np.float64)
        return Vector2(m)

    @staticmethod
    def from_point(p: Point2) -> "Vector2":
        return Vector2(p._matrix)

    def x(self) -> float:
        return self._matrix[0]

    def y(self) -> float:
        return self._matrix[1]

    @staticmethod
    def from_ros_msg(m: geometry_msgs.Vector3) -> "Vector2":
        return Vector2.new(m.x, m.y)

    def to_ros_msg(self) -> geometry_msgs.Vector3:
        return geometry_msgs.Vector3(x=self.x(), y=self.y())


class Transform2D:
    # Matrix with shape (3, 3)
    _matrix: NDArray

    def __init__(self, matrix: NDArray) -> None:
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
