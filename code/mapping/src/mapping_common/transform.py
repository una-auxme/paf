from abc import ABC
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from geometry_msgs import msg as geometry_msgs
from mapping import msg


@dataclass(init=False, eq=False)
class _Coord2(ABC):
    """Homogenous 2 dimensional coordinate"""

    # Matrix with shape (3)
    _matrix: npt.NDArray[np.float64]

    def __init__(self, matrix: npt.NDArray[np.float64]) -> None:
        assert matrix.shape == (
            3,
        ), f"{type(self).__name__} matrix must have shape (3,)"
        self._matrix = matrix

    def x(self) -> float:
        return self._matrix[0]

    def y(self) -> float:
        return self._matrix[1]

    def __eq__(self, value) -> bool:
        if type(self) is type(value):
            return (self._matrix == value._matrix).all()
        return False


@dataclass(init=False, eq=False)
class Point2(_Coord2):
    """2 dimensional point.

    Receives both rotation and translation when transformed with a Transform2D"""

    @staticmethod
    def new(x: float, y: float) -> "Point2":
        m = np.array([x, y, 1.0], dtype=np.float64)
        return Point2(m)

    @staticmethod
    def zero() -> "Point2":
        return Point2.new(0.0, 0.0)

    @staticmethod
    def from_vector(v: "Vector2") -> "Point2":
        return Point2(v._matrix)

    @staticmethod
    def from_ros_msg(m: geometry_msgs.Point) -> "Point2":
        return Point2.new(m.x, m.y)

    def to_ros_msg(self) -> geometry_msgs.Point:
        return geometry_msgs.Point(x=self.x(), y=self.y(), z=0.0)


@dataclass(init=False, eq=False)
class Vector2(_Coord2):
    """2 dimensional direction vector.

    Receives only the rotation when transformed with a Transform2D"""

    def length(self) -> float:
        raise NotImplementedError

    def angle_to(self, other: "Vector2") -> float:
        raise NotImplementedError

    @staticmethod
    def new(x: float, y: float) -> "Vector2":
        m = np.array([x, y, 1.0], dtype=np.float64)
        return Vector2(m)

    @staticmethod
    def zero() -> "Vector2":
        return Vector2.new(0.0, 0.0)

    @staticmethod
    def from_point(p: Point2) -> "Vector2":
        return Vector2(p._matrix)

    @staticmethod
    def from_ros_msg(m: geometry_msgs.Vector3) -> "Vector2":
        return Vector2.new(m.x, m.y)

    def to_ros_msg(self) -> geometry_msgs.Vector3:
        return geometry_msgs.Vector3(x=self.x(), y=self.y(), z=0.0)


@dataclass(init=False, eq=False)
class Transform2D:
    """Homogeneous 2 dimensional transformation matrix

    Based on https://alexsm.com/homogeneous-transforms/
    """

    # Matrix with shape (3, 3)
    _matrix: npt.NDArray[np.float64]

    def __init__(self, matrix: npt.NDArray[np.float64]) -> None:
        assert matrix.shape == (
            3,
            3,
        ), "Transformation matrix must be a homogenous 3x3 matrix"
        self._matrix = matrix

    @staticmethod
    def identity() -> "Transform2D":
        return Transform2D(matrix=np.eye(3, dtype=np.float64))

    @staticmethod
    def new_rotation(angle: float) -> "Transform2D":
        """Returns a transformation matrix consisting of a rotation around `angle`

        Args:
            angle (float): Rotation angle in radians TODO: define if angle is cw or ccw
        """
        c = np.cos(angle)
        s = np.sin(angle)

        rot_matrix = np.array([[c, -s], [s, c]])
        transform = Transform2D.identity()
        transform._matrix[:2, :2] = rot_matrix
        return transform

    @staticmethod
    def new_translation(v: Vector2) -> "Transform2D":
        """Returns a transformation matrix consisting of a translation along `v`

        Args:
            v (Vector2): Translation vector
        """
        transform = Transform2D.identity()
        transform._matrix[:2, 2] = v._matrix[:2]
        return transform

    @staticmethod
    def new_rotation_translation(angle: float, v: Vector2) -> "Transform2D":
        """Returns a transformation matrix consisting of first a rotation around `angle`
        and then a translation along `v`.

        Args:
            angle (float): Rotation angle in radians
            v (Vector2): Translation vector
        """
        transform = Transform2D.new_rotation(angle)
        transform._matrix[:2, 2] = v._matrix[:2]
        return transform

    @staticmethod
    def from_ros_msg(m: msg.Transform2D) -> "Transform2D":
        matrix = np.array(m.matrix, dtype=np.float64).reshape((3, 3), order="C")
        return Transform2D(matrix)

    def to_ros_msg(self) -> msg.Transform2D:
        t = tuple(self._matrix.flatten(order="C"))
        return msg.Transform2D(matrix=t)

    def __mul__(self, other):
        if isinstance(other, Transform2D):
            return Transform2D(np.matmul(self._matrix, other._matrix))
        if isinstance(other, Point2):
            return Point2(np.matmul(self._matrix, other._matrix))
        if isinstance(other, Vector2):
            matrix = self._matrix.copy()
            matrix[:2, 2] = 0.0
            return Vector2(np.matmul(matrix, other._matrix))
        raise TypeError(
            f"Unsupported operand types for *: '{type(self)}' and '{type(other)}'"
        )

    def __eq__(self, value) -> bool:
        if type(self) is type(value):
            return (self._matrix == value._matrix).all()
        return False
