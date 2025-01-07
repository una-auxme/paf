from dataclasses import dataclass
from typing import Optional

import rospy

from .transform import Transform2D
from mapping import msg

from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


@dataclass
class Shape2D:
    """A 2 dimensional shape

    This base class should be abstract,
    but cython does not support the ABC superclass and decorators
    """

    offset: Transform2D
    """Local transformation of this shape based on
    the transformation of the entity it is attached to"""

    @staticmethod
    def from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        """Creates a shape from m

        Note that the returned shape will be a subclass of Shape2D
        """
        shape_type = None
        msg_type_lower = m.type_name.lower()
        if msg_type_lower in _shape_supported_classes_dict:
            shape_type = _shape_supported_classes_dict[msg_type_lower]
        if shape_type is None:
            rospy.logerr(
                f"Received shape type '{m.type_name}' is not supported."
                f"'Circle' shape with radius 0.5 m will be used instead."
                f"The type must be one of {_shape_supported_classes_dict.keys()}"
            )
            return Circle(radius=0.5)

        return shape_type._from_ros_msg(m)

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        raise NotImplementedError

    def to_ros_msg(self) -> msg.Shape2D:
        type_name = type(self).__name__
        return msg.Shape2D(type_name=type_name, offset=self.offset.to_ros_msg())

    def to_marker(self, transform: Transform2D) -> Marker:
        """Creates an ROS marker based on this shape

        Args:
            transform: The global transform this marker should be placed at

        Returns:
            Marker: ROS marker message
        """
        m = Marker()
        shape_transform: Transform2D = transform * self.offset
        transl = shape_transform.translation()

        m.pose.position.x = transl.x()
        m.pose.position.y = transl.y()
        m.pose.position.z = 0.0
        (
            m.pose.orientation.x,
            m.pose.orientation.y,
            m.pose.orientation.z,
            m.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, shape_transform.rotation())

        m.scale.z = 1.0
        return m


@dataclass(init=False)
class Rectangle(Shape2D):
    """Rectangle with width and height in meters"""

    length: float
    width: float

    def __init__(
        self,
        length: float,
        width: float,
        offset: Optional[Transform2D] = None,
    ):
        if offset is None:
            offset = Transform2D.identity()
        super().__init__(offset=offset)
        self.length = length
        self.width = width

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        assert (
            len(m.dimensions) == 2
        ), "Rectangle expects 2 dimensions: length and width"
        return Rectangle(
            length=m.dimensions[0],
            width=m.dimensions[1],
            offset=Transform2D.from_ros_msg(m.offset),
        )

    def to_ros_msg(self) -> msg.Shape2D:
        m = super().to_ros_msg()
        m.dimensions = [self.length, self.width]
        return m

    def to_marker(self, transform: Transform2D) -> Marker:
        m = super().to_marker(transform)
        m.type = Marker.CUBE
        m.scale.x = self.length
        m.scale.y = self.width

        return m


@dataclass(init=False)
class Circle(Shape2D):
    """Circle with radius in meters"""

    radius: float

    def __init__(self, radius: float, offset: Optional[Transform2D] = None):
        if offset is None:
            offset = Transform2D.identity()
        super().__init__(offset=offset)
        self.radius = radius

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        assert len(m.dimensions) == 1, "Circle expects one dimension: radius"
        return Circle(
            radius=m.dimensions[0],
            offset=Transform2D.from_ros_msg(m.offset),
        )

    def to_ros_msg(self) -> msg.Shape2D:
        m = super().to_ros_msg()
        m.dimensions = [self.radius]
        return m

    def to_marker(self, transform: Transform2D) -> Marker:
        m = super().to_marker(transform)
        m.type = Marker.CYLINDER
        m.scale.x = self.radius * 2.0
        m.scale.y = self.radius * 2.0

        return m


_shape_supported_classes = [Rectangle, Circle]
_shape_supported_classes_dict = {}
for t in _shape_supported_classes:
    t_name = t.__name__.lower()
    _shape_supported_classes_dict[t_name] = t
