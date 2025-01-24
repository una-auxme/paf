from dataclasses import dataclass
from typing import List, Optional

import shapely
import math

import rospy
from mapping import msg
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .transform import Transform2D, Point2, Vector2

CIRCLE_APPROXIMATION_LENGTH = 0.5


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

    def to_shapely(self, transform: Transform2D) -> shapely.Polygon:
        """Creates a shapely.Polygon based on this shape

        Args:
            transform (Transform2D): Transforms the resulting Polygon

        Returns:
            Polygon
        """
        raise NotImplementedError


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

    def to_shapely(self, transform: Transform2D) -> shapely.Polygon:
        shape_transform: Transform2D = transform * self.offset
        half_length = self.length / 2.0
        half_width = self.width / 2.0
        # LeftBack, RightBack, RightFront, LeftFront
        corners = [
            Point2.new(-half_length, half_width),
            Point2.new(-half_length, -half_width),
            Point2.new(half_length, -half_width),
            Point2.new(half_length, half_width),
        ]
        corners = [(shape_transform * p).to_shapely() for p in corners]
        return shapely.Polygon(corners)


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

    def to_shapely(self, transform: Transform2D) -> shapely.Polygon:
        shape_transform: Transform2D = transform * self.offset
        p: Point2 = Point2.from_vector(shape_transform.translation())

        outline_length = 2 * self.radius * math.pi
        segments = outline_length / CIRCLE_APPROXIMATION_LENGTH
        quad_segs: int = max(2, int(segments / 4))
        return shapely.buffer(
            p.to_shapely(), self.radius, quad_segs=quad_segs, cap_style="round"
        )


@dataclass(init=False)
class Polygon(Shape2D):
    """Polygon defined by a list of Point2 objects."""

    # The points attribute does not have a redundant point for start and end
    points: List[Point2]

    def __init__(self, points: List[Point2], offset: Optional[Transform2D] = None):
        assert len(points) >= 3, "Polygon requires at least 3 points."
        if offset is None:
            offset = Transform2D.identity()
        super().__init__(offset=offset)
        self.points = points

    @staticmethod
    def _from_ros_msg(m: msg.Shape2D) -> "Shape2D":
        assert len(m.dimensions) >= 6 and (
            len(m.dimensions) % 2 == 0
        ), "Polygon requires at least 3 points."
        # Convert the flat list into Point2 objects
        points = [
            Point2.new(m.dimensions[i], m.dimensions[i + 1])
            for i in range(0, len(m.dimensions), 2)
        ]
        return Polygon(points=points, offset=Transform2D.from_ros_msg(m.offset))

    def to_ros_msg(self) -> msg.Shape2D:
        m = super().to_ros_msg()
        dimensions = []
        for p in self.points:
            dimensions.append(p.x())
            dimensions.append(p.y())
        m.dimensions = dimensions
        return m

    def to_marker(self, transform: Transform2D) -> Marker:
        """Convert to a visualization Marker for RViz."""
        m = super().to_marker(transform)
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.05  # Line thickness

        # Initialize m.points as an empty list
        m.points = []

        # Transform and add points
        for pt in self.points:
            p = Point()
            p.x = pt.x()
            p.y = pt.y()
            p.z = 0.0
            m.points.append(p)

        # Close the polygon loop
        if len(m.points) > 0:
            m.points.append(m.points[0])

        return m

    def to_shapely(self, transform: Transform2D) -> shapely.Polygon:
        shape_transform: Transform2D = transform * self.offset
        poly_points = [(shape_transform * p).to_shapely() for p in self.points]
        return shapely.Polygon(poly_points)

    @staticmethod
    def from_shapely(poly: shapely.Polygon, make_centered: bool = False) -> "Polygon":
        """Creates a Polygon from a shapely.Polygon

        If make_centered is True, the zero-point of the resulting polygon points
        will be the centroid of poly.
        And the offset will be the translation of the centroid.

        If make_centered is False, the points will match the points of poly
        and no offset will be applied.

        Returns:
            Polygon
        """
        coords = poly.exterior.coords
        assert len(coords) >= 3, "Polygon requires at least 3 points."
        coords = coords[:-1]
        if make_centered:
            center = poly.centroid
            c_vec = Vector2.new(center.x, center.y)
            transform = Transform2D.new_translation(c_vec)
        else:
            c_vec = Vector2.zero()
            transform = Transform2D.identity()

        points = [Point2.new(x, y) - c_vec for (x, y) in coords]
        return Polygon(points, offset=transform)


_shape_supported_classes = [Rectangle, Circle, Polygon]
_shape_supported_classes_dict = {}
for t in _shape_supported_classes:
    t_name = t.__name__.lower()
    _shape_supported_classes_dict[t_name] = t
