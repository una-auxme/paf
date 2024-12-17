from dataclasses import dataclass

import rospy

from mapping import msg

from visualization_msgs.msg import Marker


@dataclass
class Shape2D:
    """A 2 dimensional shape

    This base class should be abstract,
    but cython does not support the ABC superclass and decorators
    """

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
                f"""Received shape type '{m.type_name}' is not supported.
'Circle' shape with radius 0.5 m will be used instead.
The type must be one of {_shape_supported_classes_dict.keys()}"""
            )
            return Circle(radius=0.5)

        return shape_type._from_ros_msg(m)

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        raise NotImplementedError

    def to_ros_msg(self) -> msg.Shape2D:
        type_name = type(self).__name__
        return msg.Shape2D(type_name=type_name)

    def to_marker(self) -> Marker:
        """Creates an ROS marker based on this shape

        Returns:
            Marker: ROS marker message
        """
        m = Marker()
        m.pose.position.z = 0.0
        m.scale.z = 1.0
        return m


@dataclass
class Rectangle(Shape2D):
    """Rectangle with width and height in meters

    It's center is based on the entity's transform this shape is attached to
    """

    length: float
    width: float

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        assert (
            len(m.dimensions) == 2
        ), "Rectangle expects 2 dimensions: length and width"
        return Rectangle(length=m.dimensions[0], width=m.dimensions[1])

    def to_ros_msg(self) -> msg.Shape2D:
        m = super().to_ros_msg()
        m.dimensions = [self.length, self.width]
        return m

    def to_marker(self) -> Marker:
        m = super().to_marker()
        m.type = Marker.CUBE
        m.scale.x = self.length
        m.scale.y = self.width

        return m


@dataclass
class Circle(Shape2D):
    """Circle with radius in meters

    It's center is based on the entity's transform this shape is attached to
    """

    radius: float

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        assert len(m.dimensions) == 1, "Circle expects one dimension: radius"
        return Circle(radius=m.dimensions[0])

    def to_ros_msg(self) -> msg.Shape2D:
        m = super().to_ros_msg()
        m.dimensions = [self.radius]
        return m

    def to_marker(self) -> Marker:
        m = super().to_marker()
        m.type = Marker.CYLINDER
        m.scale.x = self.radius * 2.0
        m.scale.y = self.radius * 2.0

        return m


_shape_supported_classes = [Rectangle, Circle]
_shape_supported_classes_dict = {}
for t in _shape_supported_classes:
    t_name = t.__name__.lower()
    _shape_supported_classes_dict[t_name] = t
