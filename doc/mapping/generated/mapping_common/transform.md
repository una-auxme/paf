<!-- markdownlint-disable -->
# Transform documentation

## Table of Contents

* [mapping\_common.transform](#mapping_common.transform)
  * [Point2](#mapping_common.transform.Point2)
    * [distance\_to](#mapping_common.transform.Point2.distance_to)
    * [vector](#mapping_common.transform.Point2.vector)
    * [vector\_to](#mapping_common.transform.Point2.vector_to)
    * [new](#mapping_common.transform.Point2.new)
    * [zero](#mapping_common.transform.Point2.zero)
    * [from\_vector](#mapping_common.transform.Point2.from_vector)
    * [from\_ros\_msg](#mapping_common.transform.Point2.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.transform.Point2.to_ros_msg)
    * [to\_shapely](#mapping_common.transform.Point2.to_shapely)
    * [\_\_add\_\_](#mapping_common.transform.Point2.__add__)
    * [\_\_sub\_\_](#mapping_common.transform.Point2.__sub__)
  * [Vector2](#mapping_common.transform.Vector2)
    * [length](#mapping_common.transform.Vector2.length)
    * [normalized](#mapping_common.transform.Vector2.normalized)
    * [angle\_to](#mapping_common.transform.Vector2.angle_to)
    * [point](#mapping_common.transform.Vector2.point)
    * [new](#mapping_common.transform.Vector2.new)
    * [zero](#mapping_common.transform.Vector2.zero)
    * [forward](#mapping_common.transform.Vector2.forward)
    * [backward](#mapping_common.transform.Vector2.backward)
    * [left](#mapping_common.transform.Vector2.left)
    * [right](#mapping_common.transform.Vector2.right)
    * [from\_point](#mapping_common.transform.Vector2.from_point)
    * [from\_ros\_msg](#mapping_common.transform.Vector2.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.transform.Vector2.to_ros_msg)
    * [\_\_mul\_\_](#mapping_common.transform.Vector2.__mul__)
    * [\_\_rmul\_\_](#mapping_common.transform.Vector2.__rmul__)
    * [\_\_truediv\_\_](#mapping_common.transform.Vector2.__truediv__)
    * [\_\_add\_\_](#mapping_common.transform.Vector2.__add__)
    * [\_\_sub\_\_](#mapping_common.transform.Vector2.__sub__)
    * [\_\_neg\_\_](#mapping_common.transform.Vector2.__neg__)
  * [Transform2D](#mapping_common.transform.Transform2D)
    * [\_\_init\_\_](#mapping_common.transform.Transform2D.__init__)
    * [translation](#mapping_common.transform.Transform2D.translation)
    * [rotation](#mapping_common.transform.Transform2D.rotation)
    * [inverse](#mapping_common.transform.Transform2D.inverse)
    * [identity](#mapping_common.transform.Transform2D.identity)
    * [new\_rotation](#mapping_common.transform.Transform2D.new_rotation)
    * [new\_translation](#mapping_common.transform.Transform2D.new_translation)
    * [new\_rotation\_translation](#mapping_common.transform.Transform2D.new_rotation_translation)
    * [from\_ros\_msg](#mapping_common.transform.Transform2D.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.transform.Transform2D.to_ros_msg)
    * [\_\_mul\_\_](#mapping_common.transform.Transform2D.__mul__)
    * [\_\_eq\_\_](#mapping_common.transform.Transform2D.__eq__)

<a id="mapping_common.transform"></a>

# mapping\_common.transform

Contains 2d transformation (position, rotation) functions

**[API documentation](/doc/mapping/generated/mapping_common/transform.md)**

Overview of the main components:
- Transform2D: Homogeneous 2 dimensional transformation matrix.
  Can represent an arbitrary chain of translations and rotations.
- Point2: Used for positions in 2d space
- Vector2: Used for directions and offsets.
  Important: Transform2D does not apply translations to the vector.
- The math operators have been overloaded to support typical operations
  between Point2, Vector2 and Transform2D

<a id="mapping_common.transform.Point2"></a>

## Point2

```python
@dataclass(init=False, eq=False)
class Point2(_Coord2)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L57)

2 dimensional point.

Receives both rotation and translation when transformed with a Transform2D

<a id="mapping_common.transform.Point2.distance_to"></a>

#### distance\_to

```python
def distance_to(other: "Point2") -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L62)

<a id="mapping_common.transform.Point2.vector"></a>

#### vector

```python
def vector() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L65)

<a id="mapping_common.transform.Point2.vector_to"></a>

#### vector\_to

```python
def vector_to(other: "Point2") -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L68)

<a id="mapping_common.transform.Point2.new"></a>

#### new

```python
@staticmethod
def new(x: float, y: float) -> "Point2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L74)

<a id="mapping_common.transform.Point2.zero"></a>

#### zero

```python
@staticmethod
def zero() -> "Point2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L79)

<a id="mapping_common.transform.Point2.from_vector"></a>

#### from\_vector

```python
@staticmethod
def from_vector(v: "Vector2") -> "Point2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L83)

<a id="mapping_common.transform.Point2.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: geometry_msgs.Point) -> "Point2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L87)

<a id="mapping_common.transform.Point2.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> geometry_msgs.Point
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L90)

<a id="mapping_common.transform.Point2.to_shapely"></a>

#### to\_shapely

```python
def to_shapely() -> shapely.Point
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L93)

<a id="mapping_common.transform.Point2.__add__"></a>

#### \_\_add\_\_

```python
def __add__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L96)

<a id="mapping_common.transform.Point2.__sub__"></a>

#### \_\_sub\_\_

```python
def __sub__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L105)

<a id="mapping_common.transform.Vector2"></a>

## Vector2

```python
@dataclass(init=False, eq=False)
class Vector2(_Coord2)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L116)

2 dimensional direction vector.

Receives only the rotation when transformed with a Transform2D

<a id="mapping_common.transform.Vector2.length"></a>

#### length

```python
def length() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L121)

Calculates the length of this vector

**Returns**:

- `float` - length of this vector

<a id="mapping_common.transform.Vector2.normalized"></a>

#### normalized

```python
def normalized() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L131)

Returns this direction Vector with length 1.0

If the vector is the zero vector, the result will be zero as well

**Returns**:

- `Vector2` - Vector with length 1.0

<a id="mapping_common.transform.Vector2.angle_to"></a>

#### angle\_to

```python
def angle_to(other: "Vector2") -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L144)

Calculates the angle to *other*

**Arguments**:

- `other` _Vector2_ - Vector to calculate the angle to
  

**Returns**:

- `float` - signed angle in radians. Always in interval [-pi,pi].
  
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Vector2.point"></a>

#### point

```python
def point() -> Point2
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L169)

<a id="mapping_common.transform.Vector2.new"></a>

#### new

```python
@staticmethod
def new(x: float, y: float) -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L173)

<a id="mapping_common.transform.Vector2.zero"></a>

#### zero

```python
@staticmethod
def zero() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L178)

<a id="mapping_common.transform.Vector2.forward"></a>

#### forward

```python
@staticmethod
def forward() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L182)

<a id="mapping_common.transform.Vector2.backward"></a>

#### backward

```python
@staticmethod
def backward() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L186)

<a id="mapping_common.transform.Vector2.left"></a>

#### left

```python
@staticmethod
def left() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L190)

<a id="mapping_common.transform.Vector2.right"></a>

#### right

```python
@staticmethod
def right() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L194)

<a id="mapping_common.transform.Vector2.from_point"></a>

#### from\_point

```python
@staticmethod
def from_point(p: Point2) -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L198)

<a id="mapping_common.transform.Vector2.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: geometry_msgs.Vector3) -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L202)

<a id="mapping_common.transform.Vector2.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> geometry_msgs.Vector3
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L205)

<a id="mapping_common.transform.Vector2.__mul__"></a>

#### \_\_mul\_\_

```python
def __mul__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L208)

<a id="mapping_common.transform.Vector2.__rmul__"></a>

#### \_\_rmul\_\_

```python
def __rmul__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L217)

<a id="mapping_common.transform.Vector2.__truediv__"></a>

#### \_\_truediv\_\_

```python
def __truediv__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L224)

<a id="mapping_common.transform.Vector2.__add__"></a>

#### \_\_add\_\_

```python
def __add__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L233)

<a id="mapping_common.transform.Vector2.__sub__"></a>

#### \_\_sub\_\_

```python
def __sub__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L246)

<a id="mapping_common.transform.Vector2.__neg__"></a>

#### \_\_neg\_\_

```python
def __neg__() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L255)

<a id="mapping_common.transform.Transform2D"></a>

## Transform2D

```python
@dataclass(init=False, eq=False)
class Transform2D()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L262)

Homogeneous 2 dimensional transformation matrix.

Can represent an arbitrary chain of translations and rotations.

Based on https://alexsm.com/homogeneous-transforms/

Multiply Transform2D with [`Vector2`](#mapping_common.transform.Vector2) or [`Point2`](#mapping_common.transform.Point2) to apply its transformation.

Further explanation:
https://en.wikipedia.org/w/index.php?title=Transformation_matrix&oldid=1275425866#Composing_and_inverting_transformations

**Examples**:

  ##### Transform a Vector2
```python
v = Vector2.new(1.0, 0.0)
t = Transform2D.new_rotation(math.pi/2.0)
v_transformed = t * v
```
  v_transformed is (0.0, 1.0)
  
  Note that Vectors are only directions/offsets and ignore translations.

<a id="mapping_common.transform.Transform2D.__init__"></a>

#### \_\_init\_\_

```python
def __init__(matrix: npt.NDArray[np.float64]) -> None
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L289)

<a id="mapping_common.transform.Transform2D.translation"></a>

#### translation

```python
def translation() -> Vector2
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L296)

Returns only the translation that this Transform applies

**Returns**:

- `Vector2` - translation

<a id="mapping_common.transform.Transform2D.rotation"></a>

#### rotation

```python
def rotation() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L306)

Returns only the rotation that this Transform applies

**Returns**:

- `float` - rotation angle in radians
  
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Transform2D.inverse"></a>

#### inverse

```python
def inverse() -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L319)

Returns an inverted Transformation matrix

**Returns**:

- `Transform2D` - Inverted Transformation matrix

<a id="mapping_common.transform.Transform2D.identity"></a>

#### identity

```python
@staticmethod
def identity() -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L328)

Returns the identity transform (no transformation)

**Returns**:

- `Transform2D` - Identity transform

<a id="mapping_common.transform.Transform2D.new_rotation"></a>

#### new\_rotation

```python
@staticmethod
def new_rotation(angle: float) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L337)

Returns a transformation matrix consisting of a rotation around `angle`

**Arguments**:

- `angle` _float_ - Rotation angle in radians
  
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Transform2D.new_translation"></a>

#### new\_translation

```python
@staticmethod
def new_translation(v: Vector2) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L355)

Returns a transformation matrix consisting of a translation along `v`

**Arguments**:

- `v` _Vector2_ - Translation vector

<a id="mapping_common.transform.Transform2D.new_rotation_translation"></a>

#### new\_rotation\_translation

```python
@staticmethod
def new_rotation_translation(angle: float, v: Vector2) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L366)

Returns a transformation matrix consisting of first a rotation around `angle`
and then a translation along `v`.

**Arguments**:

- `angle` _float_ - Rotation angle in radians
  - angle > 0: CCW
  - angle < 0: CW
  
- `v` _Vector2_ - Translation vector

<a id="mapping_common.transform.Transform2D.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.Transform2D) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L382)

<a id="mapping_common.transform.Transform2D.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Transform2D
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L386)

<a id="mapping_common.transform.Transform2D.__mul__"></a>

#### \_\_mul\_\_

```python
def __mul__(other)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L390)

<a id="mapping_common.transform.Transform2D.__eq__"></a>

#### \_\_eq\_\_

```python
def __eq__(value) -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L430)

