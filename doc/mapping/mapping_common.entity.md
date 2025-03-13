# mapping_common.entity module

<a id="mapping_common.entity.Motion2D"></a>

### *class* mapping_common.entity.Motion2D(linear_motion=<factory>, angular_velocity=0.0)

Bases: `object`

Motion of an entity

Speeds are global (not relative to hero car).

<a id="mapping_common.entity.Motion2D.linear_motion"></a>

#### linear_motion *: [`Vector2`](mapping_common.transform.md#mapping_common.transform.Vector2)*

Linear motion

Direction vector based on the x-axis/heading of the entity
(which is furthermore based on the entity’s transform)

The length of the vector is the velocity in m/s

<a id="mapping_common.entity.Motion2D.angular_velocity"></a>

#### angular_velocity *: `float`* *= 0.0*

Angular velocity in radians/s

- angle > 0: CCW
- angle < 0: CW

<a id="mapping_common.entity.Motion2D.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Motion2D`](#mapping_common.entity.Motion2D)

<a id="mapping_common.entity.Motion2D.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Motion2D`

<a id="mapping_common.entity.Motion2D.__init__"></a>

#### \_\_init_\_(linear_motion=<factory>, angular_velocity=0.0)

<a id="mapping_common.entity.Flags"></a>

### *class* mapping_common.entity.Flags(is_collider=False, is_stopmark=False, is_lanemark=False, is_ignored=False, is_hero=False)

Bases: `object`

Dedicated flags an entity can have.

Look into the FlagFilter class for an explanation fo the individual flags.

Note that attributes should not be accessed directly,
but only the matches_filter function should be used

<a id="mapping_common.entity.Flags.__init__"></a>

#### \_\_init_\_(is_collider=False, is_stopmark=False, is_lanemark=False, is_ignored=False, is_hero=False)

<a id="mapping_common.entity.Flags.matches_filter"></a>

#### matches_filter(f)

Returns if these Flags match the filter mask f

* **Return type:**
  `bool`

Args:
: f (FlagFilter): Filter mask to filter with

Returns:
: bool: If f matches self

<a id="mapping_common.entity.Flags.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Flags`](#mapping_common.entity.Flags)

<a id="mapping_common.entity.Flags.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Flags`

<a id="mapping_common.entity.FlagFilter"></a>

### *class* mapping_common.entity.FlagFilter(has_motion=None, is_collider=None, is_tracked=None, is_stopmark=None, is_lanemark=None, is_ignored=None, is_hero=None)

Bases: `object`

Filter mask to filter entities by their flags

Setting a flag to None means it is ignored when filtering

<a id="mapping_common.entity.FlagFilter.has_motion"></a>

#### has_motion *: `Optional`[`bool`]* *= None*

motion is not None

<a id="mapping_common.entity.FlagFilter.is_collider"></a>

#### is_collider *: `Optional`[`bool`]* *= None*

hero can collide with this entity

<a id="mapping_common.entity.FlagFilter.is_tracked"></a>

#### is_tracked *: `Optional`[`bool`]* *= None*

tracking_info is not None

<a id="mapping_common.entity.FlagFilter.is_stopmark"></a>

#### is_stopmark *: `Optional`[`bool`]* *= None*

entity is a stopmark/-line where hero has to stop and wait

<a id="mapping_common.entity.FlagFilter.is_lanemark"></a>

#### is_lanemark *: `Optional`[`bool`]* *= None*

entity is a lanemark

<a id="mapping_common.entity.FlagFilter.is_ignored"></a>

#### is_ignored *: `Optional`[`bool`]* *= None*

entity should be ignored in all downstream algorithms

<a id="mapping_common.entity.FlagFilter.is_hero"></a>

#### is_hero *: `Optional`[`bool`]* *= None*

this entity is the SssssssssssssssssssssuperCar

<a id="mapping_common.entity.FlagFilter.__init__"></a>

#### \_\_init_\_(has_motion=None, is_collider=None, is_tracked=None, is_stopmark=None, is_lanemark=None, is_ignored=None, is_hero=None)

<a id="mapping_common.entity.TrackingInfo"></a>

### *class* mapping_common.entity.TrackingInfo(visibility_time=<factory>, invisibility_time=<factory>, visibility_frame_count=0, invisibility_frame_count=0, moving_time=<factory>, standing_time=<factory>, moving_time_sum=<factory>, standing_time_sum=<factory>, min_linear_speed=0.0, max_linear_speed=0.0)

Bases: `object`

Information that might be required to consistently track entities

<a id="mapping_common.entity.TrackingInfo.visibility_time"></a>

#### visibility_time *: `Duration`*

How long the entity has been visible for. Never gets reset

<a id="mapping_common.entity.TrackingInfo.invisibility_time"></a>

#### invisibility_time *: `Duration`*

How long the entity has been uninterruptedly not visible.
Reset when the entity is visible again

<a id="mapping_common.entity.TrackingInfo.visibility_frame_count"></a>

#### visibility_frame_count *: `int`* *= 0*

In how many data frames the entity was visible. Never gets reset

<a id="mapping_common.entity.TrackingInfo.invisibility_frame_count"></a>

#### invisibility_frame_count *: `int`* *= 0*

In how many consecutive data frames the entity was not visible.
Reset when the entity is visible again

<a id="mapping_common.entity.TrackingInfo.moving_time"></a>

#### moving_time *: `Duration`*

How long an entity was moving continuously. Reset when standing

<a id="mapping_common.entity.TrackingInfo.standing_time"></a>

#### standing_time *: `Duration`*

How long an entity stood still continuously. Reset when moving

<a id="mapping_common.entity.TrackingInfo.moving_time_sum"></a>

#### moving_time_sum *: `Duration`*

Sums of all the time the entity was moving. Never gets reset

<a id="mapping_common.entity.TrackingInfo.standing_time_sum"></a>

#### standing_time_sum *: `Duration`*

Sums of all the time the entity was standing still. Never gets reset

<a id="mapping_common.entity.TrackingInfo.min_linear_speed"></a>

#### min_linear_speed *: `float`* *= 0.0*

Minimum linear speed of this entity ever recorded

<a id="mapping_common.entity.TrackingInfo.max_linear_speed"></a>

#### max_linear_speed *: `float`* *= 0.0*

Maximum linear speed of this entity ever recorded

<a id="mapping_common.entity.TrackingInfo.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`TrackingInfo`](#mapping_common.entity.TrackingInfo)

<a id="mapping_common.entity.TrackingInfo.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `TrackingInfo`

<a id="mapping_common.entity.TrackingInfo.__init__"></a>

#### \_\_init_\_(visibility_time=<factory>, invisibility_time=<factory>, visibility_frame_count=0, invisibility_frame_count=0, moving_time=<factory>, standing_time=<factory>, moving_time_sum=<factory>, standing_time_sum=<factory>, min_linear_speed=0.0, max_linear_speed=0.0)

<a id="mapping_common.entity.Entity"></a>

### *class* mapping_common.entity.Entity(confidence, priority, shape, transform, timestamp=<factory>, flags=<factory>, uuid=<factory>, sensor_id=<factory>, motion=None, tracking_info=None)

Bases: `object`

A thing of interest around the hero car that has a location and a shape

<a id="mapping_common.entity.Entity.confidence"></a>

#### confidence *: `float`*

The sensor’s confidence that this entity is correct and actually exists.

<a id="mapping_common.entity.Entity.priority"></a>

#### priority *: `float`*

This entity’s priority over others. Mostly relevant for the merging filter

<a id="mapping_common.entity.Entity.shape"></a>

#### shape *: [`Shape2D`](mapping_common.shape.md#mapping_common.shape.Shape2D)*

Shape2D for collision calculations

<a id="mapping_common.entity.Entity.transform"></a>

#### transform *: [`Transform2D`](mapping_common.transform.md#mapping_common.transform.Transform2D)*

Transform2D based on the map origin (hero car)

<a id="mapping_common.entity.Entity.timestamp"></a>

#### timestamp *: `Time`*

When adding the entity its timestamp is the timestamp
of the associated sensor data
(might slightly differ to the timestamp of the Map)

In the filtering steps the entity will be predicted to the map creation timestamp.
All entities of the output map have the same timestamp as the map.

<a id="mapping_common.entity.Entity.flags"></a>

#### flags *: [`Flags`](#mapping_common.entity.Flags)*

Flags (Categories) this entity is in

Filter for them with the matches_filter(f) function and a FlagFilter

<a id="mapping_common.entity.Entity.uuid"></a>

#### uuid *: `UUID`*

Unique id of the entity in the map, set by the map

If the entity is tracked, this id is persistent across map data frames

<a id="mapping_common.entity.Entity.sensor_id"></a>

#### sensor_id *: `List`[`str`]*

Sort-of unique id(s) set by the sensor

Used to aid the tracking.

Prefixed with the sensor node name to make sure ids stay unique between sensors.

Mainly set from the tracking id outputs of the yolov11 model.

<a id="mapping_common.entity.Entity.motion"></a>

#### motion *: `Optional`[[`Motion2D`](#mapping_common.entity.Motion2D)]* *= None*

The motion of this entity

If unset the motion is unknown (Entity not properly tracked yet),
but not necessarily zero.
Some algorithms might assume the motion is zero when unset.

<a id="mapping_common.entity.Entity.tracking_info"></a>

#### tracking_info *: `Optional`[[`TrackingInfo`](#mapping_common.entity.TrackingInfo)]* *= None*

Hold the tracking information for this entity

If this entity is supposed to be tracked, tracking_info must not be None

<a id="mapping_common.entity.Entity.matches_filter"></a>

#### matches_filter(f)

Returns if this entity matches the filter mask f

* **Return type:**
  `bool`

Args:
: f (FlagFilter): Filter mask to filter with

Returns:
: bool: If f matches self

<a id="mapping_common.entity.Entity.from_ros_msg"></a>

#### *static* from_ros_msg(m)

Creates an entity from m

Note that the returned entity might be a subclass of Entity

* **Return type:**
  [`Entity`](#mapping_common.entity.Entity)

<a id="mapping_common.entity.Entity.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Entity`

<a id="mapping_common.entity.Entity.to_marker"></a>

#### to_marker()

Creates an ROS marker based on the entity

* **Return type:**
  `Marker`

Returns:
: Marker: ROS marker message

<a id="mapping_common.entity.Entity.get_meta_markers"></a>

#### get_meta_markers()

Creates additional meta markers for the entity

* **Return type:**
  `List`[`Marker`]

Returns:
: List[Marker]: List of ROS marker messages

<a id="mapping_common.entity.Entity.to_motion_marker"></a>

#### to_motion_marker()

* **Return type:**
  `Marker`

<a id="mapping_common.entity.Entity.get_text_marker"></a>

#### get_text_marker(text, offset=None)

* **Return type:**
  `Marker`

<a id="mapping_common.entity.Entity.to_shapely"></a>

#### to_shapely()

* **Return type:**
  [`ShapelyEntity`](#mapping_common.entity.ShapelyEntity)

<a id="mapping_common.entity.Entity.is_mergeable_with"></a>

#### is_mergeable_with(other)

Returns if self should be merged/combined with other at all

Mainly used in the filtering steps of the intermediate layer map

* **Return type:**
  `bool`

Args:
: other (Entity): Other entity to merge with

Returns:
: bool: If self and other should be merged at all

<a id="mapping_common.entity.Entity.get_global_x_velocity"></a>

#### get_global_x_velocity()

Returns the global x velocity of the entity in in m/s.

Returns:
- Optional[float]: Velocity of the entity in front in m/s.

* **Return type:**
  `Optional`[`float`]

<a id="mapping_common.entity.Entity.get_delta_forward_velocity_of"></a>

#### get_delta_forward_velocity_of(other)

* **Return type:**
  `Optional`[`float`]

Calculates the delta velocity compared to other in the heading of self.
: This function is only for objects in front. If the entity is behind this
  value has to be inverted. Use the “get_delta_velocity_of” function for this
  case.

- result > 0: other moves in the forward direction of self
- result < 0: other moves in the backward direction of self

Args:
: other (Entity)

Returns:
: Optional[float]: Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_delta_velocity_of"></a>

#### get_delta_velocity_of(other)

Calculates the delta velocity compared to other in the heading of self
:rtype: `Optional`[`float`]

- result > 0: other moves away from self
- result < 0: other moves nearer to self

Args:
: other (Entity)

Returns:
: Optional[float]: Delta velocity if both entities have one.

<a id="mapping_common.entity.Entity.get_width"></a>

#### get_width()

Returns the local width (y-bounds) of the entity

* **Return type:**
  `float`

Returns:
: float: width

<a id="mapping_common.entity.Entity.get_front_x"></a>

#### get_front_x()

Returns the local x length from the center to the front of the entity

* **Return type:**
  `float`

Returns:
: float: width

<a id="mapping_common.entity.Entity.__init__"></a>

#### \_\_init_\_(confidence, priority, shape, transform, timestamp=<factory>, flags=<factory>, uuid=<factory>, sensor_id=<factory>, motion=None, tracking_info=None)

<a id="mapping_common.entity.Car"></a>

### *class* mapping_common.entity.Car(brake_light=BrakeLightState.OFF, indicator=IndicatorState.OFF, \*\*kwargs)

Bases: [`Entity`](#mapping_common.entity.Entity)

<a id="mapping_common.entity.Car.BrakeLightState"></a>

#### *class* BrakeLightState(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.entity.Car.BrakeLightState.OFF"></a>

#### OFF *= 0*

<a id="mapping_common.entity.Car.BrakeLightState.ON"></a>

#### ON *= 1*

<a id="mapping_common.entity.Car.IndicatorState"></a>

#### *class* IndicatorState(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.entity.Car.IndicatorState.OFF"></a>

#### OFF *= 0*

<a id="mapping_common.entity.Car.IndicatorState.LEFT"></a>

#### LEFT *= 1*

<a id="mapping_common.entity.Car.IndicatorState.RIGHT"></a>

#### RIGHT *= 2*

<a id="mapping_common.entity.Car.__init__"></a>

#### \_\_init_\_(brake_light=BrakeLightState.OFF, indicator=IndicatorState.OFF, \*\*kwargs)

<a id="mapping_common.entity.Car.brake_light"></a>

#### brake_light *: [`BrakeLightState`](#mapping_common.entity.Car.BrakeLightState)*

<a id="mapping_common.entity.Car.indicator"></a>

#### indicator *: [`IndicatorState`](#mapping_common.entity.Car.IndicatorState)*

<a id="mapping_common.entity.Car.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Entity`

<a id="mapping_common.entity.Car.to_marker"></a>

#### to_marker()

Creates an ROS marker based on the entity

* **Return type:**
  `Marker`

Returns:
: Marker: ROS marker message

<a id="mapping_common.entity.Lanemarking"></a>

### *class* mapping_common.entity.Lanemarking(style, position_index, predicted, \*\*kwargs)

Bases: [`Entity`](#mapping_common.entity.Entity)

<a id="mapping_common.entity.Lanemarking.Style"></a>

#### *class* Style(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.entity.Lanemarking.Style.SOLID"></a>

#### SOLID *= 0*

<a id="mapping_common.entity.Lanemarking.Style.DASHED"></a>

#### DASHED *= 1*

<a id="mapping_common.entity.Lanemarking.__init__"></a>

#### \_\_init_\_(style, position_index, predicted, \*\*kwargs)

<a id="mapping_common.entity.Lanemarking.style"></a>

#### style *: [`Style`](#mapping_common.entity.Lanemarking.Style)*

<a id="mapping_common.entity.Lanemarking.position_index"></a>

#### position_index *: `int`*

<a id="mapping_common.entity.Lanemarking.predicted"></a>

#### predicted *: `bool`*

<a id="mapping_common.entity.Lanemarking.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Entity`

<a id="mapping_common.entity.Lanemarking.to_marker"></a>

#### to_marker()

Creates an ROS marker based on the entity

* **Return type:**
  `Marker`

Returns:
: Marker: ROS marker message

<a id="mapping_common.entity.Lanemarking.get_meta_markers"></a>

#### get_meta_markers()

Creates additional meta markers for the entity

* **Return type:**
  `List`[`Marker`]

Returns:
: List[Marker]: List of ROS marker messages

<a id="mapping_common.entity.t"></a>

### mapping_common.entity.t

alias of [`Pedestrian`](#mapping_common.entity.Pedestrian)

<a id="mapping_common.entity.TrafficLight"></a>

### *class* mapping_common.entity.TrafficLight(state, \*\*kwargs)

Bases: [`Entity`](#mapping_common.entity.Entity)

Traffic light or stop sign

Note: Class may be split up later

TrafficLight and StopSign add only their stop line to the map.
They set the *is_stopmark* flag only if the car has to stop there.

<a id="mapping_common.entity.TrafficLight.State"></a>

#### *class* State(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.entity.TrafficLight.State.RED"></a>

#### RED *= 0*

<a id="mapping_common.entity.TrafficLight.State.GREEN"></a>

#### GREEN *= 1*

<a id="mapping_common.entity.TrafficLight.State.YELLOW"></a>

#### YELLOW *= 2*

<a id="mapping_common.entity.TrafficLight.__init__"></a>

#### \_\_init_\_(state, \*\*kwargs)

<a id="mapping_common.entity.TrafficLight.state"></a>

#### state *: [`State`](#mapping_common.entity.TrafficLight.State)*

<a id="mapping_common.entity.TrafficLight.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Entity`

<a id="mapping_common.entity.StopMark"></a>

### *class* mapping_common.entity.StopMark(reason, \*\*kwargs)

Bases: [`Entity`](#mapping_common.entity.Entity)

Stop mark as a virtual obstacle for the ACC

<a id="mapping_common.entity.StopMark.__init__"></a>

#### \_\_init_\_(reason, \*\*kwargs)

<a id="mapping_common.entity.StopMark.reason"></a>

#### reason *: `str`*

<a id="mapping_common.entity.StopMark.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Entity`

<a id="mapping_common.entity.StopMark.to_marker"></a>

#### to_marker()

Creates an ROS marker based on the entity

* **Return type:**
  `Marker`

Returns:
: Marker: ROS marker message

<a id="mapping_common.entity.StopMark.get_meta_markers"></a>

#### get_meta_markers()

Creates additional meta markers for the entity

* **Return type:**
  `List`[`Marker`]

Returns:
: List[Marker]: List of ROS marker messages

<a id="mapping_common.entity.Pedestrian"></a>

### *class* mapping_common.entity.Pedestrian(\*\*kwargs)

Bases: [`Entity`](#mapping_common.entity.Entity)

<a id="mapping_common.entity.Pedestrian.confidence"></a>

#### confidence *: `float`*

The sensor’s confidence that this entity is correct and actually exists.

<a id="mapping_common.entity.Pedestrian.priority"></a>

#### priority *: `float`*

This entity’s priority over others. Mostly relevant for the merging filter

<a id="mapping_common.entity.Pedestrian.shape"></a>

#### shape *: [`Shape2D`](mapping_common.shape.md#mapping_common.shape.Shape2D)*

Shape2D for collision calculations

<a id="mapping_common.entity.Pedestrian.transform"></a>

#### transform *: [`Transform2D`](mapping_common.transform.md#mapping_common.transform.Transform2D)*

Transform2D based on the map origin (hero car)

<a id="mapping_common.entity.Pedestrian.timestamp"></a>

#### timestamp *: `Time`*

When adding the entity its timestamp is the timestamp
of the associated sensor data
(might slightly differ to the timestamp of the Map)

In the filtering steps the entity will be predicted to the map creation timestamp.
All entities of the output map have the same timestamp as the map.

<a id="mapping_common.entity.Pedestrian.flags"></a>

#### flags *: [`Flags`](#mapping_common.entity.Flags)*

Flags (Categories) this entity is in

Filter for them with the matches_filter(f) function and a FlagFilter

<a id="mapping_common.entity.Pedestrian.uuid"></a>

#### uuid *: `UUID`*

Unique id of the entity in the map, set by the map

If the entity is tracked, this id is persistent across map data frames

<a id="mapping_common.entity.Pedestrian.sensor_id"></a>

#### sensor_id *: `List`[`str`]*

Sort-of unique id(s) set by the sensor

Used to aid the tracking.

Prefixed with the sensor node name to make sure ids stay unique between sensors.

Mainly set from the tracking id outputs of the yolov11 model.

<a id="mapping_common.entity.Pedestrian.__init__"></a>

#### \_\_init_\_(\*\*kwargs)

<a id="mapping_common.entity.Pedestrian.to_marker"></a>

#### to_marker()

Creates an ROS marker based on the entity

* **Return type:**
  `Marker`

Returns:
: Marker: ROS marker message

<a id="mapping_common.entity.ShapelyEntity"></a>

### *class* mapping_common.entity.ShapelyEntity(entity, poly)

Bases: `object`

A Container containing both an entity and a shapely.Polygon
based on the entity’s shape and transform

**IMPORTANT** If the entity is modified, the Polygon will
not automatically update itself and will contain outdated information.

<a id="mapping_common.entity.ShapelyEntity.__init__"></a>

#### \_\_init_\_(entity, poly)

<a id="mapping_common.entity.ShapelyEntity.entity"></a>

#### entity *: [`Entity`](#mapping_common.entity.Entity)*

<a id="mapping_common.entity.ShapelyEntity.poly"></a>

#### poly *: `Polygon`*

<a id="mapping_common.entity.ShapelyEntity.get_distance_to"></a>

#### get_distance_to(other)

Returns the distance to other in m.

* **Return type:**
  `float`

Args:
: other (ShapelyEntity)

Returns:
: float: distance
