<!-- markdownlint-disable -->
# Table of Contents

* [mapping\_common.entity](#mapping_common.entity)
  * [Motion2D](#mapping_common.entity.Motion2D)
    * [linear\_motion](#mapping_common.entity.Motion2D.linear_motion)
    * [angular\_velocity](#mapping_common.entity.Motion2D.angular_velocity)
  * [Flags](#mapping_common.entity.Flags)
    * [matches\_filter](#mapping_common.entity.Flags.matches_filter)
  * [FlagFilter](#mapping_common.entity.FlagFilter)
    * [has\_motion](#mapping_common.entity.FlagFilter.has_motion)
    * [is\_collider](#mapping_common.entity.FlagFilter.is_collider)
    * [is\_tracked](#mapping_common.entity.FlagFilter.is_tracked)
    * [is\_stopmark](#mapping_common.entity.FlagFilter.is_stopmark)
    * [is\_lanemark](#mapping_common.entity.FlagFilter.is_lanemark)
    * [is\_ignored](#mapping_common.entity.FlagFilter.is_ignored)
    * [is\_hero](#mapping_common.entity.FlagFilter.is_hero)
  * [TrackingInfo](#mapping_common.entity.TrackingInfo)
    * [visibility\_time](#mapping_common.entity.TrackingInfo.visibility_time)
    * [invisibility\_time](#mapping_common.entity.TrackingInfo.invisibility_time)
    * [visibility\_frame\_count](#mapping_common.entity.TrackingInfo.visibility_frame_count)
    * [invisibility\_frame\_count](#mapping_common.entity.TrackingInfo.invisibility_frame_count)
    * [moving\_time](#mapping_common.entity.TrackingInfo.moving_time)
    * [standing\_time](#mapping_common.entity.TrackingInfo.standing_time)
    * [moving\_time\_sum](#mapping_common.entity.TrackingInfo.moving_time_sum)
    * [standing\_time\_sum](#mapping_common.entity.TrackingInfo.standing_time_sum)
    * [min\_linear\_speed](#mapping_common.entity.TrackingInfo.min_linear_speed)
    * [max\_linear\_speed](#mapping_common.entity.TrackingInfo.max_linear_speed)
  * [Entity](#mapping_common.entity.Entity)
    * [confidence](#mapping_common.entity.Entity.confidence)
    * [priority](#mapping_common.entity.Entity.priority)
    * [shape](#mapping_common.entity.Entity.shape)
    * [transform](#mapping_common.entity.Entity.transform)
    * [timestamp](#mapping_common.entity.Entity.timestamp)
    * [flags](#mapping_common.entity.Entity.flags)
    * [uuid](#mapping_common.entity.Entity.uuid)
    * [sensor\_id](#mapping_common.entity.Entity.sensor_id)
    * [motion](#mapping_common.entity.Entity.motion)
    * [tracking\_info](#mapping_common.entity.Entity.tracking_info)
    * [matches\_filter](#mapping_common.entity.Entity.matches_filter)
    * [from\_ros\_msg](#mapping_common.entity.Entity.from_ros_msg)
    * [to\_marker](#mapping_common.entity.Entity.to_marker)
    * [get\_meta\_markers](#mapping_common.entity.Entity.get_meta_markers)
    * [is\_mergeable\_with](#mapping_common.entity.Entity.is_mergeable_with)
    * [get\_global\_x\_velocity](#mapping_common.entity.Entity.get_global_x_velocity)
    * [get\_delta\_forward\_velocity\_of](#mapping_common.entity.Entity.get_delta_forward_velocity_of)
    * [get\_delta\_velocity\_of](#mapping_common.entity.Entity.get_delta_velocity_of)
    * [get\_width](#mapping_common.entity.Entity.get_width)
    * [get\_front\_x](#mapping_common.entity.Entity.get_front_x)
  * [Lanemarking](#mapping_common.entity.Lanemarking)
    * [to\_marker](#mapping_common.entity.Lanemarking.to_marker)
  * [TrafficLight](#mapping_common.entity.TrafficLight)
  * [StopMark](#mapping_common.entity.StopMark)
  * [ShapelyEntity](#mapping_common.entity.ShapelyEntity)
    * [get\_distance\_to](#mapping_common.entity.ShapelyEntity.get_distance_to)

<a id="mapping_common.entity"></a>

# mapping\_common.entity

Contains entity-related functions

**[API documentation](/doc/mapping/generated/mapping_common/entity.md)**

<a id="mapping_common.entity.Motion2D"></a>

## Motion2D

```python
@dataclass
class Motion2D()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L26)

Motion of an entity

Speeds are global (not relative to hero car).

<a id="mapping_common.entity.Motion2D.linear_motion"></a>

#### linear\_motion: `Vector2`

Linear motion

Direction vector based on the x-axis/heading of the entity
(which is furthermore based on the entity's transform)

The length of the vector is the velocity in m/s

<a id="mapping_common.entity.Motion2D.angular_velocity"></a>

#### angular\_velocity: `float`

Angular velocity in radians/s

- angle > 0: CCW
- angle < 0: CW

<a id="mapping_common.entity.Flags"></a>

## Flags

```python
@dataclass(init=False)
class Flags()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L60)

Dedicated flags an entity can have.

Look into the FlagFilter class for an explanation fo the individual flags.

Note that attributes should not be accessed directly,
but only the matches_filter function should be used

<a id="mapping_common.entity.Flags.matches_filter"></a>

#### matches\_filter(f: "FlagFilter")

```python
def matches_filter(f: "FlagFilter") -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L89)

Returns if these Flags match the filter mask f

**Arguments**:

- `f` _FlagFilter_ - Filter mask to filter with
  

**Returns**:

- `bool` - If f matches self

<a id="mapping_common.entity.FlagFilter"></a>

## FlagFilter

```python
@dataclass
class FlagFilter()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L136)

Filter mask to filter entities by their flags

Setting a flag to None means it is ignored when filtering

<a id="mapping_common.entity.FlagFilter.has_motion"></a>

#### has\_motion: `Optional[bool]`

motion is not None

<a id="mapping_common.entity.FlagFilter.is_collider"></a>

#### is\_collider: `Optional[bool]`

hero can collide with this entity

<a id="mapping_common.entity.FlagFilter.is_tracked"></a>

#### is\_tracked: `Optional[bool]`

tracking_info is not None

<a id="mapping_common.entity.FlagFilter.is_stopmark"></a>

#### is\_stopmark: `Optional[bool]`

entity is a stopmark/-line where hero has to stop and wait

<a id="mapping_common.entity.FlagFilter.is_lanemark"></a>

#### is\_lanemark: `Optional[bool]`

entity is a lanemark

<a id="mapping_common.entity.FlagFilter.is_ignored"></a>

#### is\_ignored: `Optional[bool]`

entity should be ignored in all downstream algorithms

<a id="mapping_common.entity.FlagFilter.is_hero"></a>

#### is\_hero: `Optional[bool]`

this entity is the SssssssssssssssssssssuperCar

<a id="mapping_common.entity.TrackingInfo"></a>

## TrackingInfo

```python
@dataclass
class TrackingInfo()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L159)

Information that might be required to consistently track entities

Note: As of 03.2025, this class and attribute in [`Entity`](#mapping_common.entity.Entity) is still completely unused.
PAF24 still left it in as a base/guidance for future tracking experiments

<a id="mapping_common.entity.TrackingInfo.visibility_time"></a>

#### visibility\_time: `Duration`

How long the entity has been visible for. Never gets reset

<a id="mapping_common.entity.TrackingInfo.invisibility_time"></a>

#### invisibility\_time: `Duration`

How long the entity has been uninterruptedly not visible.
Reset when the entity is visible again

<a id="mapping_common.entity.TrackingInfo.visibility_frame_count"></a>

#### visibility\_frame\_count: `int`

In how many data frames the entity was visible. Never gets reset

<a id="mapping_common.entity.TrackingInfo.invisibility_frame_count"></a>

#### invisibility\_frame\_count: `int`

In how many consecutive data frames the entity was not visible.
Reset when the entity is visible again

<a id="mapping_common.entity.TrackingInfo.moving_time"></a>

#### moving\_time: `Duration`

How long an entity was moving continuously. Reset when standing

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.standing_time"></a>

#### standing\_time: `Duration`

How long an entity stood still continuously. Reset when moving

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.moving_time_sum"></a>

#### moving\_time\_sum: `Duration`

Sums of all the time the entity was moving. Never gets reset

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.standing_time_sum"></a>

#### standing\_time\_sum: `Duration`

Sums of all the time the entity was standing still. Never gets reset

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.min_linear_speed"></a>

#### min\_linear\_speed: `float`

Minimum linear speed of this entity ever recorded

<a id="mapping_common.entity.TrackingInfo.max_linear_speed"></a>

#### max\_linear\_speed: `float`

Maximum linear speed of this entity ever recorded

<a id="mapping_common.entity.Entity"></a>

## Entity

```python
@dataclass
class Entity()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L232)

A thing of interest around the hero car that has a location and a shape

<a id="mapping_common.entity.Entity.confidence"></a>

#### confidence: `float`

The sensor's confidence that this entity is correct and actually exists.

<a id="mapping_common.entity.Entity.priority"></a>

#### priority: `float`

This entity's priority over others. Mostly relevant for the merging filter

<a id="mapping_common.entity.Entity.shape"></a>

#### shape: `Shape2D`

Shape2D for collision calculations

<a id="mapping_common.entity.Entity.transform"></a>

#### transform: `Transform2D`

Transform2D based on the map origin (hero car)

<a id="mapping_common.entity.Entity.timestamp"></a>

#### timestamp: `Time`

When adding the entity its timestamp is the timestamp
of the associated sensor data
(might slightly differ to the timestamp of the Map)

In the filtering steps the entity will be predicted to the map creation timestamp.
All entities of the output map have the same timestamp as the map.

<a id="mapping_common.entity.Entity.flags"></a>

#### flags: `Flags`

Flags (Categories) this entity is in

Filter for them with the matches_filter(f) function and a FlagFilter

<a id="mapping_common.entity.Entity.uuid"></a>

#### uuid: `UUID`

Unique id of the entity in the map, set by the map

If the entity is tracked, this id is persistent across map data frames

<a id="mapping_common.entity.Entity.sensor_id"></a>

#### sensor\_id: `List[str]`

Sort-of unique id(s) set by the sensor

Used to aid the tracking.

Prefixed with the sensor node name to make sure ids stay unique between sensors.

Mainly set from the tracking id outputs of the yolov11 model.

<a id="mapping_common.entity.Entity.motion"></a>

#### motion: `Optional[Motion2D]`

The motion of this entity

If unset the motion is unknown (Entity not properly tracked yet),
but not necessarily zero.
Some algorithms might assume the motion is zero when unset.

<a id="mapping_common.entity.Entity.tracking_info"></a>

#### tracking\_info: `Optional[TrackingInfo]`

Hold the tracking information for this entity

If this entity is supposed to be tracked, tracking_info must not be None

<a id="mapping_common.entity.Entity.matches_filter"></a>

#### matches\_filter(f: FlagFilter)

```python
def matches_filter(f: FlagFilter) -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L282)

Returns if this entity matches the filter mask f

**Arguments**:

- `f` _FlagFilter_ - Filter mask to filter with
  

**Returns**:

- `bool` - If f matches self

<a id="mapping_common.entity.Entity.from_ros_msg"></a>

#### from\_ros\_msg(m: msg.Entity)

```python
@staticmethod
def from_ros_msg(m: msg.Entity) -> "Entity"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L302)

Creates an entity from m

Note that the returned entity might be a subclass of Entity

<a id="mapping_common.entity.Entity.to_marker"></a>

#### to\_marker()

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L378)

Creates an ROS marker based on the entity

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.entity.Entity.get_meta_markers"></a>

#### get\_meta\_markers()

```python
def get_meta_markers() -> List[Marker]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L393)

Creates additional meta markers for the entity

**Returns**:

- `List[Marker]` - List of ROS marker messages

<a id="mapping_common.entity.Entity.is_mergeable_with"></a>

#### is\_mergeable\_with(other: "Entity")

```python
def is_mergeable_with(other: "Entity") -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L452)

Returns if self should be merged/combined with other at all

Mainly used in the filtering steps of the intermediate layer map

**Arguments**:

- `other` _Entity_ - Other entity to merge with
  

**Returns**:

- `bool` - If self and other should be merged at all

<a id="mapping_common.entity.Entity.get_global_x_velocity"></a>

#### get\_global\_x\_velocity()

```python
def get_global_x_velocity() -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L486)

Returns the global x velocity of the entity in in m/s.

Note that *global* is a bit misleading in this case.
It does NOT mean global in relation to the hero's GPS coordinates,
but in the x-direction of the map this entity belongs to.

-> It returns the velocity in the same x-direction as the hero.

**Returns**:

  - Optional[float]: Velocity of the entity in front in m/s.

<a id="mapping_common.entity.Entity.get_delta_forward_velocity_of"></a>

#### get\_delta\_forward\_velocity\_of(other: "Entity")

```python
def get_delta_forward_velocity_of(other: "Entity") -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L509)

Calculates the delta velocity compared to other in the heading of self.
This function is only for objects in front. If the entity is behind this
value has to be inverted. Use the "get_delta_velocity_of" function for this
case.

- result > 0: other moves in the forward direction of self
- result < 0: other moves in the backward direction of self

**Arguments**:

  other (Entity)
  

**Returns**:

- `Optional[float]` - Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_delta_velocity_of"></a>

#### get\_delta\_velocity\_of(other: "Entity")

```python
def get_delta_velocity_of(other: "Entity") -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L534)

Calculates the delta velocity compared to other in the heading of self

- result > 0: other moves away from self
- result < 0: other moves nearer to self

**Arguments**:

  other (Entity)
  

**Returns**:

- `Optional[float]` - Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_width"></a>

#### get\_width()

```python
def get_width() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L557)

Returns the local width (y-bounds) of the entity

**Returns**:

- `float` - width

<a id="mapping_common.entity.Entity.get_front_x"></a>

#### get\_front\_x()

```python
def get_front_x() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L567)

Returns the local x length from the center to the front of the entity

**Returns**:

- `float` - width

<a id="mapping_common.entity.Lanemarking"></a>

## Lanemarking

```python
@dataclass(init=False)
class Lanemarking(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L626)

<a id="mapping_common.entity.Lanemarking.to_marker"></a>

#### to\_marker()

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L660)

Creates an ROS marker based on the entity

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.entity.TrafficLight"></a>

## TrafficLight

```python
@dataclass(init=False)
class TrafficLight(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L691)

Traffic light or stop sign

Note: Class may be split up later

TrafficLight and StopSign add only their stop line to the map.
They set the *is_stopmark* flag only if the car has to stop there.

<a id="mapping_common.entity.StopMark"></a>

## StopMark

```python
@dataclass(init=False)
class StopMark(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L724)

Stop mark as a virtual obstacle for the ACC

<a id="mapping_common.entity.ShapelyEntity"></a>

## ShapelyEntity

```python
@dataclass
class ShapelyEntity()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L797)

A Container containing both an entity and a shapely.Polygon
based on the entity's shape and transform

**IMPORTANT** If the entity is modified, the Polygon will
not automatically update itself and will contain outdated information.

<a id="mapping_common.entity.ShapelyEntity.get_distance_to"></a>

#### get\_distance\_to(other: "ShapelyEntity")

```python
def get_distance_to(other: "ShapelyEntity") -> float
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/entity.py#L808)

Returns the distance to other in m.

**Arguments**:

  other (ShapelyEntity)
  

**Returns**:

- `float` - distance

