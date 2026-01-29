<!-- markdownlint-disable -->
# Shape documentation

## Table of Contents

* [mapping\_common.shape](#mapping_common.shape)
  * [CIRCLE\_APPROXIMATION\_LENGTH](#mapping_common.shape.CIRCLE_APPROXIMATION_LENGTH)
  * [MarkerStyle](#mapping_common.shape.MarkerStyle)
    * [CLOSED\_OBJECT](#mapping_common.shape.MarkerStyle.CLOSED_OBJECT)
    * [OPENED\_OBJECT](#mapping_common.shape.MarkerStyle.OPENED_OBJECT)
    * [LINESTRING](#mapping_common.shape.MarkerStyle.LINESTRING)
  * [Shape2D](#mapping_common.shape.Shape2D)
    * [offset](#mapping_common.shape.Shape2D.offset)
    * [from\_ros\_msg](#mapping_common.shape.Shape2D.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.shape.Shape2D.to_ros_msg)
    * [to\_marker](#mapping_common.shape.Shape2D.to_marker)
    * [to\_shapely](#mapping_common.shape.Shape2D.to_shapely)
  * [Rectangle](#mapping_common.shape.Rectangle)
    * [length](#mapping_common.shape.Rectangle.length)
    * [width](#mapping_common.shape.Rectangle.width)
    * [\_\_init\_\_](#mapping_common.shape.Rectangle.__init__)
    * [to\_ros\_msg](#mapping_common.shape.Rectangle.to_ros_msg)
    * [to\_marker](#mapping_common.shape.Rectangle.to_marker)
    * [to\_shapely](#mapping_common.shape.Rectangle.to_shapely)
  * [Circle](#mapping_common.shape.Circle)
    * [radius](#mapping_common.shape.Circle.radius)
    * [\_\_init\_\_](#mapping_common.shape.Circle.__init__)
    * [to\_ros\_msg](#mapping_common.shape.Circle.to_ros_msg)
    * [to\_marker](#mapping_common.shape.Circle.to_marker)
    * [to\_shapely](#mapping_common.shape.Circle.to_shapely)
  * [Polygon](#mapping_common.shape.Polygon)
    * [points](#mapping_common.shape.Polygon.points)
    * [\_\_init\_\_](#mapping_common.shape.Polygon.__init__)
    * [to\_ros\_msg](#mapping_common.shape.Polygon.to_ros_msg)
    * [to\_marker](#mapping_common.shape.Polygon.to_marker)
    * [to\_shapely](#mapping_common.shape.Polygon.to_shapely)
    * [from\_shapely](#mapping_common.shape.Polygon.from_shapely)

<a id="mapping_common.shape"></a>

# mapping\_common.shape

Contains shape classes and functions

**[API documentation](/doc/mapping/generated/mapping_common/shape.md)**

Overview of the main components:
- Abstract Shape2D base class. Subclasses: **Rectangle, Circle, Polygon**
- Used to define the shape of entities in the Intermediate Layer
- Shape calculations:
  - For algorithms on shapes, the
    **[shapely](https://shapely.readthedocs.io/en/stable/manual.html)** library
    is used across the project.
  - The Shape2D classes are interoperable with **shapely** via their
    `Shape2D.to_shapely()` and `Polygon.from_shapely()` methods.

<a id="mapping_common.shape.CIRCLE_APPROXIMATION_LENGTH"></a>

#### CIRCLE\_APPROXIMATION\_LENGTH

Precision when converting a Circle into a shapely.Polygon

<a id="mapping_common.shape.MarkerStyle"></a>

## MarkerStyle

```python
class MarkerStyle(Enum)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L37)

<a id="mapping_common.shape.MarkerStyle.CLOSED_OBJECT"></a>

#### CLOSED\_OBJECT

<a id="mapping_common.shape.MarkerStyle.OPENED_OBJECT"></a>

#### OPENED\_OBJECT

<a id="mapping_common.shape.MarkerStyle.LINESTRING"></a>

#### LINESTRING

<a id="mapping_common.shape.Shape2D"></a>

## Shape2D

```python
@dataclass
class Shape2D()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L44)

A 2 dimensional shape

This base class should be abstract,
but cython does not support the ABC superclass and decorators

<a id="mapping_common.shape.Shape2D.offset"></a>

#### offset: `Transform2D`

Local transformation of this shape

If this shape is attached to an entity, this acts as an additional offset
on top of the entity's transformation.

<a id="mapping_common.shape.Shape2D.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.Shape2D) -> "Shape2D"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L58)

Creates a shape from m

Note that the returned shape will be a subclass of Shape2D

<a id="mapping_common.shape.Shape2D.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Shape2D
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L81)

<a id="mapping_common.shape.Shape2D.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L85)

Creates an ROS marker based on this shape

**Arguments**:

- `transform` - The global transform this marker should be placed at
  

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.shape.Shape2D.to_shapely"></a>

#### to\_shapely

```python
def to_shapely(transform: Optional[Transform2D] = None) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L117)

Creates a shapely.Polygon based on this shape

**Arguments**:

- `transform` _Transform2D_ - Transforms the resulting Polygon
  

**Returns**:

- `Polygon` - shapely Polygon

<a id="mapping_common.shape.Rectangle"></a>

## Rectangle

```python
@dataclass(init=False)
class Rectangle(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L130)

Rectangle with width and height in meters

<a id="mapping_common.shape.Rectangle.length"></a>

#### length: `float`

<a id="mapping_common.shape.Rectangle.width"></a>

#### width: `float`

<a id="mapping_common.shape.Rectangle.__init__"></a>

#### \_\_init\_\_

```python
def __init__(length: float,
             width: float,
             offset: Optional[Transform2D] = None)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L136)

<a id="mapping_common.shape.Rectangle.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Shape2D
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L159)

<a id="mapping_common.shape.Rectangle.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L164)

<a id="mapping_common.shape.Rectangle.to_shapely"></a>

#### to\_shapely

```python
def to_shapely(transform: Optional[Transform2D] = None) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L176)

<a id="mapping_common.shape.Circle"></a>

## Circle

```python
@dataclass(init=False)
class Circle(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L194)

Circle with radius in meters

<a id="mapping_common.shape.Circle.radius"></a>

#### radius: `float`

<a id="mapping_common.shape.Circle.__init__"></a>

#### \_\_init\_\_

```python
def __init__(radius: float, offset: Optional[Transform2D] = None)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L199)

<a id="mapping_common.shape.Circle.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Shape2D
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L213)

<a id="mapping_common.shape.Circle.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L218)

<a id="mapping_common.shape.Circle.to_shapely"></a>

#### to\_shapely

```python
def to_shapely(transform: Optional[Transform2D] = None) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L230)

<a id="mapping_common.shape.Polygon"></a>

## Polygon

```python
@dataclass(init=False)
class Polygon(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L245)

Polygon defined by a list of Point2 objects.

<a id="mapping_common.shape.Polygon.points"></a>

#### points: `List[Point2]`

Polygon points

The list does NOT have a redundant point for start and end.

<a id="mapping_common.shape.Polygon.__init__"></a>

#### \_\_init\_\_

```python
def __init__(points: List[Point2], offset: Optional[Transform2D] = None)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L254)

<a id="mapping_common.shape.Polygon.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Shape2D
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L273)

<a id="mapping_common.shape.Polygon.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L282)

Convert to a visualization Marker for RViz.

<a id="mapping_common.shape.Polygon.to_shapely"></a>

#### to\_shapely

```python
def to_shapely(transform: Optional[Transform2D] = None) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L381)

<a id="mapping_common.shape.Polygon.from_shapely"></a>

#### from\_shapely

```python
@staticmethod
def from_shapely(poly: shapely.Polygon,
                 make_centered: bool = False) -> "Polygon"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/shape.py#L390)

Creates a Polygon from a shapely.Polygon

If make_centered is True, the zero-point of the resulting polygon points
will be the centroid of poly.
And the offset will be the translation of the centroid.

If make_centered is False, the points will match the points of poly
and no offset will be applied.

**Arguments**:

- `poly` _shapely.Polygon_ - poly
- `make_centered` _bool, optional_ - Center the polygon points.
  Defaults to False.
  

**Returns**:

- `Polygon` - Polygon

