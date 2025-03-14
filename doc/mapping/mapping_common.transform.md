# mapping_common.transform module

- [mapping_common.transform module]()
  - [`Point2`](#mapping_common.transform.Point2)
    - [`Point2.distance_to()`](#mapping_common.transform.Point2.distance_to)
    - [`Point2.vector()`](#mapping_common.transform.Point2.vector)
    - [`Point2.vector_to()`](#mapping_common.transform.Point2.vector_to)
    - [`Point2.new()`](#mapping_common.transform.Point2.new)
    - [`Point2.zero()`](#mapping_common.transform.Point2.zero)
    - [`Point2.from_vector()`](#mapping_common.transform.Point2.from_vector)
    - [`Point2.from_ros_msg()`](#mapping_common.transform.Point2.from_ros_msg)
    - [`Point2.to_ros_msg()`](#mapping_common.transform.Point2.to_ros_msg)
    - [`Point2.to_shapely()`](#mapping_common.transform.Point2.to_shapely)
  - [`Vector2`](#mapping_common.transform.Vector2)
    - [`Vector2.length()`](#mapping_common.transform.Vector2.length)
    - [`Vector2.normalized()`](#mapping_common.transform.Vector2.normalized)
    - [`Vector2.angle_to()`](#mapping_common.transform.Vector2.angle_to)
    - [`Vector2.point()`](#mapping_common.transform.Vector2.point)
    - [`Vector2.new()`](#mapping_common.transform.Vector2.new)
    - [`Vector2.zero()`](#mapping_common.transform.Vector2.zero)
    - [`Vector2.forward()`](#mapping_common.transform.Vector2.forward)
    - [`Vector2.backward()`](#mapping_common.transform.Vector2.backward)
    - [`Vector2.left()`](#mapping_common.transform.Vector2.left)
    - [`Vector2.right()`](#mapping_common.transform.Vector2.right)
    - [`Vector2.from_point()`](#mapping_common.transform.Vector2.from_point)
    - [`Vector2.from_ros_msg()`](#mapping_common.transform.Vector2.from_ros_msg)
    - [`Vector2.to_ros_msg()`](#mapping_common.transform.Vector2.to_ros_msg)
  - [`Transform2D`](#mapping_common.transform.Transform2D)
    - [`Transform2D.__init__()`](#mapping_common.transform.Transform2D.__init__)
    - [`Transform2D.translation()`](#mapping_common.transform.Transform2D.translation)
    - [`Transform2D.rotation()`](#mapping_common.transform.Transform2D.rotation)
    - [`Transform2D.inverse()`](#mapping_common.transform.Transform2D.inverse)
    - [`Transform2D.identity()`](#mapping_common.transform.Transform2D.identity)
    - [`Transform2D.new_rotation()`](#mapping_common.transform.Transform2D.new_rotation)
    - [`Transform2D.new_translation()`](#mapping_common.transform.Transform2D.new_translation)
    - [`Transform2D.new_rotation_translation()`](#mapping_common.transform.Transform2D.new_rotation_translation)
    - [`Transform2D.from_ros_msg()`](#mapping_common.transform.Transform2D.from_ros_msg)
    - [`Transform2D.to_ros_msg()`](#mapping_common.transform.Transform2D.to_ros_msg)

<a id="mapping_common.transform.Point2"></a>

### *class* mapping_common.transform.Point2(matrix)

Bases: `_Coord2`

2 dimensional point.

Receives both rotation and translation when transformed with a Transform2D

<a id="mapping_common.transform.Point2.distance_to"></a>

#### distance_to(other)

* **Return type:**
  `float`

<a id="mapping_common.transform.Point2.vector"></a>

#### vector()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Point2.vector_to"></a>

#### vector_to(other)

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Point2.new"></a>

#### *static* new(x, y)

* **Return type:**
  [`Point2`](#mapping_common.transform.Point2)

<a id="mapping_common.transform.Point2.zero"></a>

#### *static* zero()

* **Return type:**
  [`Point2`](#mapping_common.transform.Point2)

<a id="mapping_common.transform.Point2.from_vector"></a>

#### *static* from_vector(v)

* **Return type:**
  [`Point2`](#mapping_common.transform.Point2)

<a id="mapping_common.transform.Point2.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Point2`](#mapping_common.transform.Point2)

<a id="mapping_common.transform.Point2.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Point`

<a id="mapping_common.transform.Point2.to_shapely"></a>

#### to_shapely()

* **Return type:**
  `Point`

<a id="mapping_common.transform.Vector2"></a>

### *class* mapping_common.transform.Vector2(matrix)

Bases: `_Coord2`

2 dimensional direction vector.

Receives only the rotation when transformed with a Transform2D

<a id="mapping_common.transform.Vector2.length"></a>

#### length()

Calculates the length of this vector

* **Return type:**
  `float`

Returns:
: float: length of this vector

<a id="mapping_common.transform.Vector2.normalized"></a>

#### normalized()

Returns this direction Vector with length 1.0

If the vector is the zero vector, the result will be zero as well

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

Returns:
: Vector2: Vector with length 1.0

<a id="mapping_common.transform.Vector2.angle_to"></a>

#### angle_to(other)

Calculates the angle to *other*

* **Return type:**
  `float`

Args:
: other (Vector2): Vector to calculate the angle to

Returns:
: float: signed angle in radians. Always in interval [-pi,pi].
  <br/>
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Vector2.point"></a>

#### point()

* **Return type:**
  [`Point2`](#mapping_common.transform.Point2)

<a id="mapping_common.transform.Vector2.new"></a>

#### *static* new(x, y)

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.zero"></a>

#### *static* zero()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.forward"></a>

#### *static* forward()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.backward"></a>

#### *static* backward()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.left"></a>

#### *static* left()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.right"></a>

#### *static* right()

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.from_point"></a>

#### *static* from_point(p)

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

<a id="mapping_common.transform.Vector2.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Vector3`

<a id="mapping_common.transform.Transform2D"></a>

### *class* mapping_common.transform.Transform2D(matrix)

Bases: `object`

Homogeneous 2 dimensional transformation matrix

Based on [https://alexsm.com/homogeneous-transforms/](https://alexsm.com/homogeneous-transforms/)

## Examples:
### Transform a Vector2
``python
v = Vector2.new(1.0, 0.0)
t = Transform2D.new_rotation(math.pi/2.0)
v_transformed = t * v
``
v_transformed is (0.0, 1.0)

Note that Vectors are only directions/offsets and ignore translations.

<a id="mapping_common.transform.Transform2D.__init__"></a>

#### \_\_init_\_(matrix)

<a id="mapping_common.transform.Transform2D.translation"></a>

#### translation()

Returns only the translation that this Transform applies

* **Return type:**
  [`Vector2`](#mapping_common.transform.Vector2)

Returns:
: Vector2: translation

<a id="mapping_common.transform.Transform2D.rotation"></a>

#### rotation()

Returns only the rotation that this Transform applies

* **Return type:**
  `float`

Returns:
: float: rotation angle in radians
  <br/>
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Transform2D.inverse"></a>

#### inverse()

Returns an inverted Transformation matrix

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

Returns:
: Transform2D: Inverted Transformation matrix

<a id="mapping_common.transform.Transform2D.identity"></a>

#### *static* identity()

Returns the identity transform (no transformation)

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

Returns:
: Transform2D: Identity transform

<a id="mapping_common.transform.Transform2D.new_rotation"></a>

#### *static* new_rotation(angle)

Returns a transformation matrix consisting of a rotation around angle

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

Args:
: angle (float): Rotation angle in radians
  <br/>
  - angle > 0: CCW
  - angle < 0: CW

<a id="mapping_common.transform.Transform2D.new_translation"></a>

#### *static* new_translation(v)

Returns a transformation matrix consisting of a translation along v

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

Args:
: v (Vector2): Translation vector

<a id="mapping_common.transform.Transform2D.new_rotation_translation"></a>

#### *static* new_rotation_translation(angle, v)

Returns a transformation matrix consisting of first a rotation around angle
and then a translation along v.

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

Args:
: angle (float): Rotation angle in radians
  - angle > 0: CCW
  - angle < 0: CW
  <br/>
  v (Vector2): Translation vector

<a id="mapping_common.transform.Transform2D.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Transform2D`](#mapping_common.transform.Transform2D)

<a id="mapping_common.transform.Transform2D.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Transform2D`
