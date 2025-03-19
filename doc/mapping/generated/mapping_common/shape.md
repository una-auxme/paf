<!-- markdownlint-disable -->
# Shape documentation

## Table of Contents

* [mapping\_common.shape](#mapping_common.shape)
  * [Shape2D](#mapping_common.shape.Shape2D)
    * [offset](#mapping_common.shape.Shape2D.offset)
    * [from\_ros\_msg](#mapping_common.shape.Shape2D.from_ros_msg)
    * [to\_marker](#mapping_common.shape.Shape2D.to_marker)
    * [to\_shapely](#mapping_common.shape.Shape2D.to_shapely)
  * [Rectangle](#mapping_common.shape.Rectangle)
  * [Circle](#mapping_common.shape.Circle)
  * [Polygon](#mapping_common.shape.Polygon)
    * [to\_marker](#mapping_common.shape.Polygon.to_marker)
    * [from\_shapely](#mapping_common.shape.Polygon.from_shapely)

<a id="mapping_common.shape"></a>

# mapping\_common.shape

Contains shape-related functions

**[API documentation](/doc/mapping/generated/mapping_common/shape.md)**

<a id="mapping_common.shape.Shape2D"></a>

## Shape2D

```python
@dataclass
class Shape2D()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L32)

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

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L46)

Creates a shape from m

Note that the returned shape will be a subclass of Shape2D

<a id="mapping_common.shape.Shape2D.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L73)

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

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L105)

Creates a shapely.Polygon based on this shape

**Arguments**:

- `transform` _Transform2D_ - Transforms the resulting Polygon
  

**Returns**:

  Polygon

<a id="mapping_common.shape.Rectangle"></a>

## Rectangle

```python
@dataclass(init=False)
class Rectangle(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L118)

Rectangle with width and height in meters

<a id="mapping_common.shape.Circle"></a>

## Circle

```python
@dataclass(init=False)
class Circle(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L182)

Circle with radius in meters

<a id="mapping_common.shape.Polygon"></a>

## Polygon

```python
@dataclass(init=False)
class Polygon(Shape2D)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L233)

Polygon defined by a list of Point2 objects.

<a id="mapping_common.shape.Polygon.to_marker"></a>

#### to\_marker

```python
def to_marker(transform: Optional[Transform2D] = None,
              marker_style: MarkerStyle = MarkerStyle.CLOSED_OBJECT) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L267)

Convert to a visualization Marker for RViz.

<a id="mapping_common.shape.Polygon.from_shapely"></a>

#### from\_shapely

```python
@staticmethod
def from_shapely(poly: shapely.Polygon,
                 make_centered: bool = False) -> "Polygon"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/shape.py#L375)

Creates a Polygon from a shapely.Polygon

If make_centered is True, the zero-point of the resulting polygon points
will be the centroid of poly.
And the offset will be the translation of the centroid.

If make_centered is False, the points will match the points of poly
and no offset will be applied.

**Returns**:

  Polygon

