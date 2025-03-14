# mapping_common.shape module

- [mapping_common.shape module]()
  - [`MarkerStyle`](#mapping_common.shape.MarkerStyle)
    - [`MarkerStyle.CLOSED_OBJECT`](#mapping_common.shape.MarkerStyle.CLOSED_OBJECT)
    - [`MarkerStyle.OPENED_OBJECT`](#mapping_common.shape.MarkerStyle.OPENED_OBJECT)
    - [`MarkerStyle.LINESTRING`](#mapping_common.shape.MarkerStyle.LINESTRING)
  - [`Shape2D`](#mapping_common.shape.Shape2D)
    - [`Shape2D.offset`](#mapping_common.shape.Shape2D.offset)
    - [`Shape2D.from_ros_msg()`](#mapping_common.shape.Shape2D.from_ros_msg)
    - [`Shape2D.to_ros_msg()`](#mapping_common.shape.Shape2D.to_ros_msg)
    - [`Shape2D.to_marker()`](#mapping_common.shape.Shape2D.to_marker)
    - [`Shape2D.to_shapely()`](#mapping_common.shape.Shape2D.to_shapely)
    - [`Shape2D.__init__()`](#mapping_common.shape.Shape2D.__init__)
  - [`Rectangle`](#mapping_common.shape.Rectangle)
    - [`Rectangle.__init__()`](#mapping_common.shape.Rectangle.__init__)
    - [`Rectangle.length`](#mapping_common.shape.Rectangle.length)
    - [`Rectangle.width`](#mapping_common.shape.Rectangle.width)
    - [`Rectangle.to_ros_msg()`](#mapping_common.shape.Rectangle.to_ros_msg)
    - [`Rectangle.to_marker()`](#mapping_common.shape.Rectangle.to_marker)
    - [`Rectangle.to_shapely()`](#mapping_common.shape.Rectangle.to_shapely)
  - [`Circle`](#mapping_common.shape.Circle)
    - [`Circle.__init__()`](#mapping_common.shape.Circle.__init__)
    - [`Circle.radius`](#mapping_common.shape.Circle.radius)
    - [`Circle.to_ros_msg()`](#mapping_common.shape.Circle.to_ros_msg)
    - [`Circle.to_marker()`](#mapping_common.shape.Circle.to_marker)
    - [`Circle.to_shapely()`](#mapping_common.shape.Circle.to_shapely)
  - [`Polygon`](#mapping_common.shape.Polygon)
    - [`Polygon.__init__()`](#mapping_common.shape.Polygon.__init__)
    - [`Polygon.points`](#mapping_common.shape.Polygon.points)
    - [`Polygon.to_ros_msg()`](#mapping_common.shape.Polygon.to_ros_msg)
    - [`Polygon.to_marker()`](#mapping_common.shape.Polygon.to_marker)
    - [`Polygon.to_shapely()`](#mapping_common.shape.Polygon.to_shapely)
    - [`Polygon.from_shapely()`](#mapping_common.shape.Polygon.from_shapely)
  - [`t`](#mapping_common.shape.t)

<a id="mapping_common.shape.MarkerStyle"></a>

### *class* mapping_common.shape.MarkerStyle(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.shape.MarkerStyle.CLOSED_OBJECT"></a>

#### CLOSED_OBJECT *= 'closed_object'*

<a id="mapping_common.shape.MarkerStyle.OPENED_OBJECT"></a>

#### OPENED_OBJECT *= 'opened_object'*

<a id="mapping_common.shape.MarkerStyle.LINESTRING"></a>

#### LINESTRING *= 'linestring'*

<a id="mapping_common.shape.Shape2D"></a>

### *class* mapping_common.shape.Shape2D(offset)

Bases: `object`

A 2 dimensional shape

This base class should be abstract,
but cython does not support the ABC superclass and decorators

<a id="mapping_common.shape.Shape2D.offset"></a>

#### offset *: [`Transform2D`](mapping_common.transform.md#mapping_common.transform.Transform2D)*

Local transformation of this shape based on
the transformation of the entity it is attached to

<a id="mapping_common.shape.Shape2D.from_ros_msg"></a>

#### *static* from_ros_msg(m)

Creates a shape from m

Note that the returned shape will be a subclass of Shape2D

* **Return type:**
  [`Shape2D`](#mapping_common.shape.Shape2D)

<a id="mapping_common.shape.Shape2D.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Shape2D`

<a id="mapping_common.shape.Shape2D.to_marker"></a>

#### to_marker(transform=None, marker_style=MarkerStyle.CLOSED_OBJECT)

Creates an ROS marker based on this shape

* **Return type:**
  `Marker`

Args:
: transform: The global transform this marker should be placed at

Returns:
: Marker: ROS marker message

<a id="mapping_common.shape.Shape2D.to_shapely"></a>

#### to_shapely(transform=None)

Creates a shapely.Polygon based on this shape

* **Return type:**
  `Polygon`

Args:
: transform (Transform2D): Transforms the resulting Polygon

Returns:
: Polygon

<a id="mapping_common.shape.Shape2D.__init__"></a>

#### \_\_init_\_(offset)

<a id="mapping_common.shape.Rectangle"></a>

### *class* mapping_common.shape.Rectangle(length, width, offset=None)

Bases: [`Shape2D`](#mapping_common.shape.Shape2D)

Rectangle with width and height in meters

<a id="mapping_common.shape.Rectangle.__init__"></a>

#### \_\_init_\_(length, width, offset=None)

<a id="mapping_common.shape.Rectangle.length"></a>

#### length *: `float`*

<a id="mapping_common.shape.Rectangle.width"></a>

#### width *: `float`*

<a id="mapping_common.shape.Rectangle.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Shape2D`

<a id="mapping_common.shape.Rectangle.to_marker"></a>

#### to_marker(transform=None, marker_style=MarkerStyle.CLOSED_OBJECT)

Creates an ROS marker based on this shape

* **Return type:**
  `Marker`

Args:
: transform: The global transform this marker should be placed at

Returns:
: Marker: ROS marker message

<a id="mapping_common.shape.Rectangle.to_shapely"></a>

#### to_shapely(transform=None)

Creates a shapely.Polygon based on this shape

* **Return type:**
  `Polygon`

Args:
: transform (Transform2D): Transforms the resulting Polygon

Returns:
: Polygon

<a id="mapping_common.shape.Circle"></a>

### *class* mapping_common.shape.Circle(radius, offset=None)

Bases: [`Shape2D`](#mapping_common.shape.Shape2D)

Circle with radius in meters

<a id="mapping_common.shape.Circle.__init__"></a>

#### \_\_init_\_(radius, offset=None)

<a id="mapping_common.shape.Circle.radius"></a>

#### radius *: `float`*

<a id="mapping_common.shape.Circle.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Shape2D`

<a id="mapping_common.shape.Circle.to_marker"></a>

#### to_marker(transform=None, marker_style=MarkerStyle.CLOSED_OBJECT)

Creates an ROS marker based on this shape

* **Return type:**
  `Marker`

Args:
: transform: The global transform this marker should be placed at

Returns:
: Marker: ROS marker message

<a id="mapping_common.shape.Circle.to_shapely"></a>

#### to_shapely(transform=None)

Creates a shapely.Polygon based on this shape

* **Return type:**
  `Polygon`

Args:
: transform (Transform2D): Transforms the resulting Polygon

Returns:
: Polygon

<a id="mapping_common.shape.Polygon"></a>

### *class* mapping_common.shape.Polygon(points, offset=None)

Bases: [`Shape2D`](#mapping_common.shape.Shape2D)

Polygon defined by a list of Point2 objects.

<a id="mapping_common.shape.Polygon.__init__"></a>

#### \_\_init_\_(points, offset=None)

<a id="mapping_common.shape.Polygon.points"></a>

#### points *: `List`[[`Point2`](mapping_common.transform.md#mapping_common.transform.Point2)]*

<a id="mapping_common.shape.Polygon.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Shape2D`

<a id="mapping_common.shape.Polygon.to_marker"></a>

#### to_marker(transform=None, marker_style=MarkerStyle.CLOSED_OBJECT)

Convert to a visualization Marker for RViz.

* **Return type:**
  `Marker`

<a id="mapping_common.shape.Polygon.to_shapely"></a>

#### to_shapely(transform=None)

Creates a shapely.Polygon based on this shape

* **Return type:**
  `Polygon`

Args:
: transform (Transform2D): Transforms the resulting Polygon

Returns:
: Polygon

<a id="mapping_common.shape.Polygon.from_shapely"></a>

#### *static* from_shapely(poly, make_centered=False)

Creates a Polygon from a shapely.Polygon

If make_centered is True, the zero-point of the resulting polygon points
will be the centroid of poly.
And the offset will be the translation of the centroid.

If make_centered is False, the points will match the points of poly
and no offset will be applied.

* **Return type:**
  [`Polygon`](#mapping_common.shape.Polygon)

Returns:
: Polygon

<a id="mapping_common.shape.t"></a>

### mapping_common.shape.t

alias of [`Polygon`](#mapping_common.shape.Polygon)
