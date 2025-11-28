<!-- markdownlint-disable -->
# Entity documentation

## Table of Contents

* [mapping\_common.entity](#mapping_common.entity)
  * [Motion2D](#mapping_common.entity.Motion2D)
    * [linear\_motion](#mapping_common.entity.Motion2D.linear_motion)
    * [angular\_velocity](#mapping_common.entity.Motion2D.angular_velocity)
    * [from\_ros\_msg](#mapping_common.entity.Motion2D.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.entity.Motion2D.to_ros_msg)
  * [Flags](#mapping_common.entity.Flags)
    * [\_\_init\_\_](#mapping_common.entity.Flags.__init__)
    * [matches\_filter](#mapping_common.entity.Flags.matches_filter)
    * [from\_ros\_msg](#mapping_common.entity.Flags.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.entity.Flags.to_ros_msg)
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
    * [from\_ros\_msg](#mapping_common.entity.TrackingInfo.from_ros_msg)
    * [to\_ros\_msg](#mapping_common.entity.TrackingInfo.to_ros_msg)
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
    * [to\_ros\_msg](#mapping_common.entity.Entity.to_ros_msg)
    * [to\_marker](#mapping_common.entity.Entity.to_marker)
    * [get\_meta\_markers](#mapping_common.entity.Entity.get_meta_markers)
    * [to\_motion\_marker](#mapping_common.entity.Entity.to_motion_marker)
    * [get\_text\_marker](#mapping_common.entity.Entity.get_text_marker)
    * [to\_shapely](#mapping_common.entity.Entity.to_shapely)
    * [is\_mergeable\_with](#mapping_common.entity.Entity.is_mergeable_with)
    * [get\_global\_x\_velocity](#mapping_common.entity.Entity.get_global_x_velocity)
    * [get\_delta\_forward\_velocity\_of](#mapping_common.entity.Entity.get_delta_forward_velocity_of)
    * [get\_delta\_velocity\_of](#mapping_common.entity.Entity.get_delta_velocity_of)
    * [get\_width](#mapping_common.entity.Entity.get_width)
    * [get\_front\_x](#mapping_common.entity.Entity.get_front_x)
  * [Car](#mapping_common.entity.Car)
    * [brake\_light](#mapping_common.entity.Car.brake_light)
    * [indicator](#mapping_common.entity.Car.indicator)
    * [BrakeLightState](#mapping_common.entity.Car.BrakeLightState)
    * [IndicatorState](#mapping_common.entity.Car.IndicatorState)
    * [\_\_init\_\_](#mapping_common.entity.Car.__init__)
    * [to\_ros\_msg](#mapping_common.entity.Car.to_ros_msg)
    * [to\_marker](#mapping_common.entity.Car.to_marker)
  * [Lanemarking](#mapping_common.entity.Lanemarking)
    * [style](#mapping_common.entity.Lanemarking.style)
    * [position\_index](#mapping_common.entity.Lanemarking.position_index)
    * [predicted](#mapping_common.entity.Lanemarking.predicted)
    * [Style](#mapping_common.entity.Lanemarking.Style)
    * [\_\_init\_\_](#mapping_common.entity.Lanemarking.__init__)
    * [to\_ros\_msg](#mapping_common.entity.Lanemarking.to_ros_msg)
    * [to\_marker](#mapping_common.entity.Lanemarking.to_marker)
    * [get\_meta\_markers](#mapping_common.entity.Lanemarking.get_meta_markers)
  * [TrafficLight](#mapping_common.entity.TrafficLight)
    * [state](#mapping_common.entity.TrafficLight.state)
    * [State](#mapping_common.entity.TrafficLight.State)
    * [\_\_init\_\_](#mapping_common.entity.TrafficLight.__init__)
    * [to\_ros\_msg](#mapping_common.entity.TrafficLight.to_ros_msg)
  * [StopMark](#mapping_common.entity.StopMark)
    * [reason](#mapping_common.entity.StopMark.reason)
    * [\_\_init\_\_](#mapping_common.entity.StopMark.__init__)
    * [to\_ros\_msg](#mapping_common.entity.StopMark.to_ros_msg)
    * [to\_marker](#mapping_common.entity.StopMark.to_marker)
    * [get\_meta\_markers](#mapping_common.entity.StopMark.get_meta_markers)
  * [Pedestrian](#mapping_common.entity.Pedestrian)
    * [\_\_init\_\_](#mapping_common.entity.Pedestrian.__init__)
    * [to\_marker](#mapping_common.entity.Pedestrian.to_marker)
  * [ShapelyEntity](#mapping_common.entity.ShapelyEntity)
    * [entity](#mapping_common.entity.ShapelyEntity.entity)
    * [poly](#mapping_common.entity.ShapelyEntity.poly)
    * [get\_distance\_to](#mapping_common.entity.ShapelyEntity.get_distance_to)

<a id="mapping_common.entity"></a>

# mapping\_common.entity

Contains Entity classes and functions

**[API documentation](/doc/mapping/generated/mapping_common/entity.md)**

Overview of the main components:
- **Entity** class and subclasses (Car, Pedestrian, etc..)
- Entity attribute classes: Motion2D, Flags, TrackingInfo
- ShapelyEntity: Container containing an Entity and its shape as shapely.Polygon

<a id="mapping_common.entity.Motion2D"></a>

## Motion2D

```python
@dataclass
class Motion2D()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L35)

Motion of an entity

Speed magnitudes are global (not relative to hero car).
The motion direction is relative as usual.

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

<a id="mapping_common.entity.Motion2D.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.Motion2D) -> "Motion2D"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L58)

<a id="mapping_common.entity.Motion2D.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Motion2D
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L62)

<a id="mapping_common.entity.Flags"></a>

## Flags

```python
@dataclass(init=False)
class Flags()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L70)

Dedicated flags an entity can have.

Look into the [`FlagFilter`](#mapping_common.entity.FlagFilter) class for an explanation of the individual flags.

Note that attributes should not be accessed directly,
but only the matches_filter function should be used

<a id="mapping_common.entity.Flags.__init__"></a>

#### \_\_init\_\_

```python
def __init__(is_collider: bool = False,
             is_stopmark: bool = False,
             is_lanemark: bool = False,
             is_ignored: bool = False,
             is_hero: bool = False)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L85)

<a id="mapping_common.entity.Flags.matches_filter"></a>

#### matches\_filter

```python
def matches_filter(f: "FlagFilter") -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L99)

Returns if these Flags match the filter mask f

**Arguments**:

- `f` _FlagFilter_ - Filter mask to filter with
  

**Returns**:

- `bool` - If f matches self

<a id="mapping_common.entity.Flags.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.Flags) -> "Flags"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L126)

<a id="mapping_common.entity.Flags.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Flags
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L135)

<a id="mapping_common.entity.FlagFilter"></a>

## FlagFilter

```python
@dataclass
class FlagFilter()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L146)

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

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L169)

Information that might be required to consistently track entities

Note: As of 03.2025, this class and attribute in [`Entity`](#mapping_common.entity.Entity) is still completely unused.
PAF24 still left it in as a base/guidance for future tracking experiments

<a id="mapping_common.entity.TrackingInfo.visibility_time"></a>

#### visibility\_time: `DurationMsg`

How long the entity has been visible for. Never gets reset

<a id="mapping_common.entity.TrackingInfo.invisibility_time"></a>

#### invisibility\_time: `DurationMsg`

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

#### moving\_time: `DurationMsg`

How long an entity was moving continuously. Reset when standing

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.standing_time"></a>

#### standing\_time: `DurationMsg`

How long an entity stood still continuously. Reset when moving

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.moving_time_sum"></a>

#### moving\_time\_sum: `DurationMsg`

Sums of all the time the entity was moving. Never gets reset

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.standing_time_sum"></a>

#### standing\_time\_sum: `DurationMsg`

Sums of all the time the entity was standing still. Never gets reset

This might be used to decide if we should overtake

<a id="mapping_common.entity.TrackingInfo.min_linear_speed"></a>

#### min\_linear\_speed: `float`

Minimum linear speed of this entity ever recorded

<a id="mapping_common.entity.TrackingInfo.max_linear_speed"></a>

#### max\_linear\_speed: `float`

Maximum linear speed of this entity ever recorded

<a id="mapping_common.entity.TrackingInfo.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.TrackingInfo) -> "TrackingInfo"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L212)

<a id="mapping_common.entity.TrackingInfo.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.TrackingInfo
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L226)

<a id="mapping_common.entity.Entity"></a>

## Entity

```python
@dataclass
class Entity()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L242)

A thing of interest around the hero car. Mainly used for Obstacles.

Has a location and a shape.

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

#### timestamp: `TimeMsg`

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

#### matches\_filter

```python
def matches_filter(f: FlagFilter) -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L295)

Returns if this entity matches the filter mask f

**Arguments**:

- `f` _FlagFilter_ - Filter mask to filter with
  

**Returns**:

- `bool` - If f matches self

<a id="mapping_common.entity.Entity.from_ros_msg"></a>

#### from\_ros\_msg

```python
@staticmethod
def from_ros_msg(m: msg.Entity) -> "Entity"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L315)

Creates an entity from m

Note that the returned entity might be a subclass of Entity

<a id="mapping_common.entity.Entity.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Entity
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L366)

<a id="mapping_common.entity.Entity.to_marker"></a>

#### to\_marker

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L392)

Creates a ROS marker based on the entity

The Marker only visualizes the transform and shape of the Entity.

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.entity.Entity.get_meta_markers"></a>

#### get\_meta\_markers

```python
def get_meta_markers() -> List[Marker]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L409)

Creates additional meta markers for the entity

**Returns**:

- `List[Marker]` - List of ROS marker messages

<a id="mapping_common.entity.Entity.to_motion_marker"></a>

#### to\_motion\_marker

```python
def to_motion_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L424)

Creates a ROS marker based on the entity's motion

**Returns**:

- `Marker` - ROS marker message
  

**Raises**:

- `Assertion` - Entity has no motion

<a id="mapping_common.entity.Entity.get_text_marker"></a>

#### get\_text\_marker

```python
def get_text_marker(text: str, offset: Optional[Vector2] = None) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L455)

Creates a text marker at the entity's position

**Arguments**:

- `text` _str_ - Text for the marker
- `offset` _Optional[Vector2], optional_ - Position offset of the marker.
  Defaults to None.
  

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.entity.Entity.to_shapely"></a>

#### to\_shapely

```python
def to_shapely() -> "ShapelyEntity"
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L483)

Calculates the [`ShapelyEntity`](#mapping_common.entity.ShapelyEntity) for this entity

**Returns**:

- `ShapelyEntity` - Container containing self and a shapely.Polygon

<a id="mapping_common.entity.Entity.is_mergeable_with"></a>

#### is\_mergeable\_with

```python
def is_mergeable_with(other: "Entity") -> bool
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L491)

Returns if self should be merged/combined with other at all

Mainly used in the filtering steps of the intermediate layer map

**Arguments**:

- `other` _Entity_ - Other entity to merge with
  

**Returns**:

- `bool` - If self and other should be merged at all

<a id="mapping_common.entity.Entity.get_global_x_velocity"></a>

#### get\_global\_x\_velocity

```python
def get_global_x_velocity() -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L525)

Returns the global x velocity of the entity in in m/s.

Note that *global* is a bit misleading in this case.
It does NOT mean global in relation to the hero's GPS coordinates,
but in the x-direction of the map this entity belongs to.

-> It returns the velocity in the same x-direction as the hero.

**Returns**:

- `Optional[float]` - Velocity of the entity in front in m/s.

<a id="mapping_common.entity.Entity.get_delta_forward_velocity_of"></a>

#### get\_delta\_forward\_velocity\_of

```python
def get_delta_forward_velocity_of(other: "Entity") -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L548)

Calculates the delta velocity compared to other in the heading of self.
This function is only for objects in front. If the entity is behind this
value has to be inverted. Use the "get_delta_velocity_of" function for this
case.

- result > 0: other moves in the forward direction of self
- result < 0: other moves in the backward direction of self

**Arguments**:

- `other` _Entity_ - other
  

**Returns**:

- `Optional[float]` - Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_delta_velocity_of"></a>

#### get\_delta\_velocity\_of

```python
def get_delta_velocity_of(other: "Entity") -> Optional[float]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L573)

Calculates the delta velocity compared to other in the heading of self

- result > 0: other moves away from self
- result < 0: other moves nearer to self

**Arguments**:

- `other` _Entity_ - other
  

**Returns**:

- `Optional[float]` - Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_width"></a>

#### get\_width

```python
def get_width() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L596)

Returns the local width (y-bounds) of the entity

**Returns**:

- `float` - width

<a id="mapping_common.entity.Entity.get_front_x"></a>

#### get\_front\_x

```python
def get_front_x() -> float
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L606)

Returns the local x length from the center to the front of the entity

**Returns**:

- `float` - width

<a id="mapping_common.entity.Car"></a>

## Car

```python
@dataclass(init=False)
class Car(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L618)

<a id="mapping_common.entity.Car.brake_light"></a>

#### brake\_light: `"Car.BrakeLightState"`

<a id="mapping_common.entity.Car.indicator"></a>

#### indicator: `"Car.IndicatorState"`

<a id="mapping_common.entity.Car.BrakeLightState"></a>

## BrakeLightState

```python
class BrakeLightState(Enum)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L622)

<a id="mapping_common.entity.Car.BrakeLightState.OFF"></a>

#### OFF

<a id="mapping_common.entity.Car.BrakeLightState.ON"></a>

#### ON

<a id="mapping_common.entity.Car.IndicatorState"></a>

## IndicatorState

```python
class IndicatorState(Enum)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L626)

<a id="mapping_common.entity.Car.IndicatorState.OFF"></a>

#### OFF

<a id="mapping_common.entity.Car.IndicatorState.LEFT"></a>

#### LEFT

<a id="mapping_common.entity.Car.IndicatorState.RIGHT"></a>

#### RIGHT

<a id="mapping_common.entity.Car.__init__"></a>

#### \_\_init\_\_

```python
def __init__(brake_light: "Car.BrakeLightState" = BrakeLightState.OFF,
             indicator: "Car.IndicatorState" = IndicatorState.OFF,
             **kwargs)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L631)

<a id="mapping_common.entity.Car.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Entity
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L648)

<a id="mapping_common.entity.Car.to_marker"></a>

#### to\_marker

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L655)

<a id="mapping_common.entity.Lanemarking"></a>

## Lanemarking

```python
@dataclass(init=False)
class Lanemarking(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L665)

<a id="mapping_common.entity.Lanemarking.style"></a>

#### style: `"Lanemarking.Style"`

<a id="mapping_common.entity.Lanemarking.position_index"></a>

#### position\_index: `int`

<a id="mapping_common.entity.Lanemarking.predicted"></a>

#### predicted: `bool`

If this Lanemark was not actually detected by a sensor
but predicted based on other/previous lanemarks

<a id="mapping_common.entity.Lanemarking.Style"></a>

## Style

```python
class Style(Enum)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L673)

<a id="mapping_common.entity.Lanemarking.Style.SOLID"></a>

#### SOLID

<a id="mapping_common.entity.Lanemarking.Style.DASHED"></a>

#### DASHED

<a id="mapping_common.entity.Lanemarking.__init__"></a>

#### \_\_init\_\_

```python
def __init__(style: "Lanemarking.Style", position_index: int, predicted: bool,
             **kwargs)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L677)

<a id="mapping_common.entity.Lanemarking.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Entity
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L693)

<a id="mapping_common.entity.Lanemarking.to_marker"></a>

#### to\_marker

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L702)

Creates an ROS marker based on the entity

**Returns**:

- `Marker` - ROS marker message

<a id="mapping_common.entity.Lanemarking.get_meta_markers"></a>

#### get\_meta\_markers

```python
def get_meta_markers() -> List[Marker]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L726)

<a id="mapping_common.entity.TrafficLight"></a>

## TrafficLight

```python
@dataclass(init=False)
class TrafficLight(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L733)

Traffic light stop line

Note: This class is currently unused. Only [`StopMark`](#mapping_common.entity.StopMark) is used at an intersection.

TrafficLight is only a stop line on the map.
It sets the *is_stopmark* flag only if the car has to stop there.

<a id="mapping_common.entity.TrafficLight.state"></a>

#### state: `"TrafficLight.State"`

<a id="mapping_common.entity.TrafficLight.State"></a>

## State

```python
class State(Enum)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L744)

<a id="mapping_common.entity.TrafficLight.State.RED"></a>

#### RED

<a id="mapping_common.entity.TrafficLight.State.GREEN"></a>

#### GREEN

<a id="mapping_common.entity.TrafficLight.State.YELLOW"></a>

#### YELLOW

<a id="mapping_common.entity.TrafficLight.__init__"></a>

#### \_\_init\_\_

```python
def __init__(state: "TrafficLight.State", **kwargs)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L749)

<a id="mapping_common.entity.TrafficLight.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Entity
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L759)

<a id="mapping_common.entity.StopMark"></a>

## StopMark

```python
@dataclass(init=False)
class StopMark(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L766)

Stop mark as a virtual obstacle for the ACC

<a id="mapping_common.entity.StopMark.reason"></a>

#### reason: `str`

Why this StopMark exits. Only for visualization.

<a id="mapping_common.entity.StopMark.__init__"></a>

#### \_\_init\_\_

```python
def __init__(reason: str, **kwargs)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L773)

<a id="mapping_common.entity.StopMark.to_ros_msg"></a>

#### to\_ros\_msg

```python
def to_ros_msg() -> msg.Entity
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L783)

<a id="mapping_common.entity.StopMark.to_marker"></a>

#### to\_marker

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L788)

<a id="mapping_common.entity.StopMark.get_meta_markers"></a>

#### get\_meta\_markers

```python
def get_meta_markers() -> List[Marker]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L798)

<a id="mapping_common.entity.Pedestrian"></a>

## Pedestrian

```python
@dataclass(init=False)
class Pedestrian(Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L813)

<a id="mapping_common.entity.Pedestrian.__init__"></a>

#### \_\_init\_\_

```python
def __init__(**kwargs)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L814)

<a id="mapping_common.entity.Pedestrian.to_marker"></a>

#### to\_marker

```python
def to_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L817)

<a id="mapping_common.entity.ShapelyEntity"></a>

## ShapelyEntity

```python
@dataclass
class ShapelyEntity()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L847)

A Container containing both an entity and a shapely.Polygon
based on the entity's shape and transform

**IMPORTANT** If the entity is modified, the Polygon will
not automatically update itself and will contain outdated information.

<a id="mapping_common.entity.ShapelyEntity.entity"></a>

#### entity: `Entity`

<a id="mapping_common.entity.ShapelyEntity.poly"></a>

#### poly: `shapely.Polygon`

<a id="mapping_common.entity.ShapelyEntity.get_distance_to"></a>

#### get\_distance\_to

```python
def get_distance_to(other: "ShapelyEntity") -> float
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/entity.py#L858)

Returns the distance to other in m.

**Arguments**:

- `other` _ShapelyEntity_ - other
  

**Returns**:

- `float` - distance

