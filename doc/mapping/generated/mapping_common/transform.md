<!-- markdownlint-disable -->
# Transform documentation

## Table of Contents

* [mapping\_common.transform](#mapping_common.transform)
  * [Point2](#mapping_common.transform.Point2)
  * [Vector2](#mapping_common.transform.Vector2)
    * [length](#mapping_common.transform.Vector2.length)
    * [normalized](#mapping_common.transform.Vector2.normalized)
    * [angle\_to](#mapping_common.transform.Vector2.angle_to)
  * [Transform2D](#mapping_common.transform.Transform2D)
    * [translation](#mapping_common.transform.Transform2D.translation)
    * [rotation](#mapping_common.transform.Transform2D.rotation)
    * [inverse](#mapping_common.transform.Transform2D.inverse)
    * [identity](#mapping_common.transform.Transform2D.identity)
    * [new\_rotation](#mapping_common.transform.Transform2D.new_rotation)
    * [new\_translation](#mapping_common.transform.Transform2D.new_translation)
    * [new\_rotation\_translation](#mapping_common.transform.Transform2D.new_rotation_translation)

<a id="mapping_common.transform"></a>

# mapping\_common.transform

Contains transform-related functions

**[API documentation](/doc/mapping/generated/mapping_common/transform.md)**

<a id="mapping_common.transform.Point2"></a>

## Point2

```python
@dataclass(init=False, eq=False)
class Point2(_Coord2)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L47)

2 dimensional point.

Receives both rotation and translation when transformed with a Transform2D

<a id="mapping_common.transform.Vector2"></a>

## Vector2

```python
@dataclass(init=False, eq=False)
class Vector2(_Coord2)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L106)

2 dimensional direction vector.

Receives only the rotation when transformed with a Transform2D

<a id="mapping_common.transform.Vector2.length"></a>

#### length

```python
def length() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L111)

Calculates the length of this vector

**Returns**:

- `float` - length of this vector

<a id="mapping_common.transform.Vector2.normalized"></a>

#### normalized

```python
def normalized() -> "Vector2"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L121)

Returns this direction Vector with length 1.0

If the vector is the zero vector, the result will be zero as well

**Returns**:

- `Vector2` - Vector with length 1.0

<a id="mapping_common.transform.Vector2.angle_to"></a>

#### angle\_to

```python
def angle_to(other: "Vector2") -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L134)

Calculates the angle to *other*

**Arguments**:

- `other` _Vector2_ - Vector to calculate the angle to
  

**Returns**:

- `float` - signed angle in radians. Always in interval [-pi,pi].
  
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Transform2D"></a>

## Transform2D

```python
@dataclass(init=False, eq=False)
class Transform2D()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L251)

Homogeneous 2 dimensional transformation matrix

Based on https://alexsm.com/homogeneous-transforms/

## Examples:
### Transform a Vector2
v_transformed is (0.0, 1.0)

Note that Vectors are only directions/offsets and ignore translations.
```python
v = Vector2.new(1.0, 0.0)
t = Transform2D.new_rotation(math.pi/2.0)
v_transformed = t * v
```

<a id="mapping_common.transform.Transform2D.translation"></a>

#### translation

```python
def translation() -> Vector2
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L278)

Returns only the translation that this Transform applies

**Returns**:

- `Vector2` - translation

<a id="mapping_common.transform.Transform2D.rotation"></a>

#### rotation

```python
def rotation() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L288)

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

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L301)

Returns an inverted Transformation matrix

**Returns**:

- `Transform2D` - Inverted Transformation matrix

<a id="mapping_common.transform.Transform2D.identity"></a>

#### identity

```python
@staticmethod
def identity() -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L310)

Returns the identity transform (no transformation)

**Returns**:

- `Transform2D` - Identity transform

<a id="mapping_common.transform.Transform2D.new_rotation"></a>

#### new\_rotation

```python
@staticmethod
def new_rotation(angle: float) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L319)

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

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L337)

Returns a transformation matrix consisting of a translation along `v`

**Arguments**:

- `v` _Vector2_ - Translation vector

<a id="mapping_common.transform.Transform2D.new_rotation_translation"></a>

#### new\_rotation\_translation

```python
@staticmethod
def new_rotation_translation(angle: float, v: Vector2) -> "Transform2D"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/transform.py#L348)

Returns a transformation matrix consisting of first a rotation around `angle`
and then a translation along `v`.

**Arguments**:

- `angle` _float_ - Rotation angle in radians
  - angle > 0: CCW
  - angle < 0: CW
  
- `v` _Vector2_ - Translation vector

