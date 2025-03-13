# mapping_common.map module

- [mapping_common.map module]()
  - [`line()`](#mapping_common.map.line)
  - [`LaneFreeState`](#mapping_common.map.LaneFreeState)
    - [`LaneFreeState.TO_BE_CHECKED`](#mapping_common.map.LaneFreeState.TO_BE_CHECKED)
    - [`LaneFreeState.FREE`](#mapping_common.map.LaneFreeState.FREE)
    - [`LaneFreeState.BLOCKED`](#mapping_common.map.LaneFreeState.BLOCKED)
    - [`LaneFreeState.MISSING_LANEMARK_ERR`](#mapping_common.map.LaneFreeState.MISSING_LANEMARK_ERR)
    - [`LaneFreeState.LANEMARK_ANGLE_ERR`](#mapping_common.map.LaneFreeState.LANEMARK_ANGLE_ERR)
    - [`LaneFreeState.SHAPE_ERR`](#mapping_common.map.LaneFreeState.SHAPE_ERR)
    - [`LaneFreeState.is_error()`](#mapping_common.map.LaneFreeState.is_error)
  - [`LaneFreeDirection`](#mapping_common.map.LaneFreeDirection)
    - [`LaneFreeDirection.LEFT`](#mapping_common.map.LaneFreeDirection.LEFT)
    - [`LaneFreeDirection.RIGHT`](#mapping_common.map.LaneFreeDirection.RIGHT)
  - [`Map`](#mapping_common.map.Map)
    - [`Map.timestamp`](#mapping_common.map.Map.timestamp)
    - [`Map.entities`](#mapping_common.map.Map.entities)
    - [`Map.hero()`](#mapping_common.map.Map.hero)
    - [`Map.entities_without_hero()`](#mapping_common.map.Map.entities_without_hero)
    - [`Map.get_lane_y_axis_intersections()`](#mapping_common.map.Map.get_lane_y_axis_intersections)
    - [`Map.build_tree()`](#mapping_common.map.Map.build_tree)
    - [`Map.filtered()`](#mapping_common.map.Map.filtered)
    - [`Map.to_multi_poly_array()`](#mapping_common.map.Map.to_multi_poly_array)
    - [`Map.from_ros_msg()`](#mapping_common.map.Map.from_ros_msg)
    - [`Map.to_ros_msg()`](#mapping_common.map.Map.to_ros_msg)
    - [`Map.__init__()`](#mapping_common.map.Map.__init__)
  - [`MapTree`](#mapping_common.map.MapTree)
    - [`MapTree.__init__()`](#mapping_common.map.MapTree.__init__)
    - [`MapTree.map`](#mapping_common.map.MapTree.map)
    - [`MapTree.filtered_entities`](#mapping_common.map.MapTree.filtered_entities)
    - [`MapTree.nearest()`](#mapping_common.map.MapTree.nearest)
    - [`MapTree.query()`](#mapping_common.map.MapTree.query)
    - [`MapTree.query_nearest()`](#mapping_common.map.MapTree.query_nearest)
    - [`MapTree.query_self()`](#mapping_common.map.MapTree.query_self)
    - [`MapTree.get_entity_in_front_or_back()`](#mapping_common.map.MapTree.get_entity_in_front_or_back)
    - [`MapTree.is_lane_free()`](#mapping_common.map.MapTree.is_lane_free)
    - [`MapTree.is_lane_free_rectangle()`](#mapping_common.map.MapTree.is_lane_free_rectangle)
    - [`MapTree.is_lane_free_lanemarking()`](#mapping_common.map.MapTree.is_lane_free_lanemarking)
    - [`MapTree.is_lane_free_intersection()`](#mapping_common.map.MapTree.is_lane_free_intersection)
    - [`MapTree.get_nearest_entity()`](#mapping_common.map.MapTree.get_nearest_entity)
    - [`MapTree.get_overlapping_entities()`](#mapping_common.map.MapTree.get_overlapping_entities)
  - [`build_global_hero_transform()`](#mapping_common.map.build_global_hero_transform)
  - [`lane_free_filter()`](#mapping_common.map.lane_free_filter)

<a id="mapping_common.map.line"></a>

### mapping_common.map.line(img, pt1, pt2, color) → img

.   @brief Draws a line segment connecting two points.
.   
.   The function line draws the line segment between pt1 and pt2 points in the image. The line is
.   clipped by the image boundaries. For non-antialiased lines with integer coordinates, the 8-connected
.   or 4-connected Bresenham algorithm is used. Thick lines are drawn with rounding endings. Antialiased
.   lines are drawn using Gaussian filtering.
.   
.   @param img Image.
.   @param pt1 First point of the line segment.
.   @param pt2 Second point of the line segment.
.   @param color Line color.
.   @param thickness Line thickness.
.   @param lineType Type of the line. See #LineTypes.
.   @param shift Number of fractional bits in the point coordinates.

<a id="mapping_common.map.LaneFreeState"></a>

### *class* mapping_common.map.LaneFreeState(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.map.LaneFreeState.TO_BE_CHECKED"></a>

#### TO_BE_CHECKED *= 2*

<a id="mapping_common.map.LaneFreeState.FREE"></a>

#### FREE *= 1*

<a id="mapping_common.map.LaneFreeState.BLOCKED"></a>

#### BLOCKED *= 0*

<a id="mapping_common.map.LaneFreeState.MISSING_LANEMARK_ERR"></a>

#### MISSING_LANEMARK_ERR *= -1*

<a id="mapping_common.map.LaneFreeState.LANEMARK_ANGLE_ERR"></a>

#### LANEMARK_ANGLE_ERR *= -2*

<a id="mapping_common.map.LaneFreeState.SHAPE_ERR"></a>

#### SHAPE_ERR *= -3*

<a id="mapping_common.map.LaneFreeState.is_error"></a>

#### is_error()

<a id="mapping_common.map.LaneFreeDirection"></a>

### *class* mapping_common.map.LaneFreeDirection(value)

Bases: `Enum`

An enumeration.

<a id="mapping_common.map.LaneFreeDirection.LEFT"></a>

#### LEFT *= False*

<a id="mapping_common.map.LaneFreeDirection.RIGHT"></a>

#### RIGHT *= True*

<a id="mapping_common.map.Map"></a>

### *class* mapping_common.map.Map(timestamp=genpy.Time[0], entities=<factory>)

Bases: `object`

2 dimensional map for the intermediate layer

General information:
- 2D top-down map. The height(z) dimension is mostly useless

> for collision detection and path planning
- The map is based on the local car position and is only built using sensor data.
  No global positioning via GPS or similar is used.
- The map (0/0) is the center of the hero car
- All transformations to entities are relative to
  the hero car’s coordinate system (position/heading)
- The map’s x-axis is aligned with the heading of the hero car
- The map’s y-axis points to the left of the hero car
- Coordinate system is a right-hand system like tf2 (can be visualized in RViz)
- The map might include the hero car as the first entity in entities

<a id="mapping_common.map.Map.timestamp"></a>

#### timestamp *: `Time`* *= genpy.Time[0]*

The timestamp this map was created at.

Should be the time when this map was initially sent off
from the mapping_data_integration node.

This timestamp is also the “freshest” compared to
the timestamps of all entities included in the map

<a id="mapping_common.map.Map.entities"></a>

#### entities *: `List`[[`Entity`](mapping_common.entity.md#mapping_common.entity.Entity)]*

The entities this map consists out of

Note that this list might also include the hero car (as first element of this list)

<a id="mapping_common.map.Map.hero"></a>

#### hero()

Returns the entity of the hero car if it is the first element of the map

* **Return type:**
  `Optional`[[`Entity`](mapping_common.entity.md#mapping_common.entity.Entity)]

Returns:
: Optional[Entity]: Entity of the hero car

<a id="mapping_common.map.Map.entities_without_hero"></a>

#### entities_without_hero()

Returns the entities without the hero car

Only checks if the first entity is_hero

* **Return type:**
  `List`[[`Entity`](mapping_common.entity.md#mapping_common.entity.Entity)]

Returns:
: List[Entity]: Entities without the hero car

<a id="mapping_common.map.Map.get_lane_y_axis_intersections"></a>

#### get_lane_y_axis_intersections(direction='left')

calculates the intersections of the lanemarks in lane_pos direction

* **Return type:**
  `dict`

Args:
: direction (str): lanemarks on “left”, “right” or “both” will be checked.
  Other inputs will be ignored

Returns:
: dict{uuid, coordinate}: dictionary with uuid of lanemark as keys
  and coordinates as according entries

<a id="mapping_common.map.Map.build_tree"></a>

#### build_tree(f=None, filter_fn=None)

Creates a filtered MapTree

**IMPORTANT**: The map
MUST NOT BE MODIFIED WHILE USING THE TREE,
otherwise results will be invalid or crash
:rtype: [`MapTree`](#mapping_common.map.MapTree)

> Useful for for quickly calculating which entities of a map are

the nearest or (intersect, touch, etc.) with a given geometry

Args:
: f (Optional[FlagFilter], optional): Filtering with FlagFilter.
  : Defaults to None.
  <br/>
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  : Filtering with function/lambda. Defaults to None.

Returns:
: MapTree

<a id="mapping_common.map.Map.filtered"></a>

#### filtered(f=None, filter_fn=None)

Filters self.entities

* **Return type:**
  `List`[[`Entity`](mapping_common.entity.md#mapping_common.entity.Entity)]

Args:
: f (Optional[FlagFilter], optional): Filtering with FlagFilter.
  : Defaults to None.
  <br/>
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  : Filtering with function/lambda. Defaults to None.

Returns:
: List[Entity]: List of entities both filters were matching for

<a id="mapping_common.map.Map.to_multi_poly_array"></a>

#### to_multi_poly_array(area_to_incorporate, resolution_scale)

Takes the entities without hero of the map and draws the contours onto
a numpy array

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`ScalarType`, bound= `generic`, covariant=True)]]

Args:
: area_to_incorporate (Tuple[int, int]): the area, in which the entities
  have to be plotted
  resolution_scale (int): since the array has only integer indices,
  scale the array

Returns:
: npt.NDArray: array with contours drawn in

<a id="mapping_common.map.Map.from_ros_msg"></a>

#### *static* from_ros_msg(m)

* **Return type:**
  [`Map`](#mapping_common.map.Map)

<a id="mapping_common.map.Map.to_ros_msg"></a>

#### to_ros_msg()

* **Return type:**
  `Map`

<a id="mapping_common.map.Map.__init__"></a>

#### \_\_init_\_(timestamp=genpy.Time[0], entities=<factory>)

<a id="mapping_common.map.MapTree"></a>

### *class* mapping_common.map.MapTree(map, f=None, filter_fn=None)

Bases: `object`

An acceleration structure around the shapely.STRtree

**IMPORTANT**: The map this tree was created with
MUST NOT BE MODIFIED WHILE USING THE TREE,
otherwise results will be invalid or crash

Useful for for quickly calculating which entities of a map are
the nearest or (intersect, touch, etc.) with a given geometry

The map’s entities can optionally be filtered upon tree creation

<a id="mapping_common.map.MapTree.__init__"></a>

#### \_\_init_\_(map, f=None, filter_fn=None)

Creates a shapely.STRtree based on the given map and filtering

Both filtering methods can be combined.
Both filters need to match for the entity to match.

Args:
: map (Map)
  f (Optional[FlagFilter], optional): Filtering with FlagFilter.
  <br/>
  > Defaults to None.
  <br/>
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  : Filtering with function/lambda. Defaults to None.

<a id="mapping_common.map.MapTree.map"></a>

#### map *: [`Map`](#mapping_common.map.Map)*

The unfiltered map this tree was created with

<a id="mapping_common.map.MapTree.filtered_entities"></a>

#### filtered_entities *: `List`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]*

Only the entities of this tree that weren’t filtered out from the map.

Also includes their shapely.Polygon

<a id="mapping_common.map.MapTree.nearest"></a>

#### nearest(geo)

Returns the nearest Entity inside the tree based on geo

More information here:
[https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.nearest](https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.nearest)

* **Return type:**
  `Optional`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]

Args:
: geo (shapely.Geometry): Geometry to calculate the nearest entity to

Returns:
: Optional[ShapelyEntity]: If the tree is empty, will return None.
  : Otherwise will return the nearest Entity

<a id="mapping_common.map.MapTree.query"></a>

#### query(geo, predicate=None, distance=None)

Calculates which entities interact with *geo*

**IMPORTANT: The query might include false positives,
because the STRtree seems to only roughly check for intersections.
It includes all entities that MIGHT intersect with geo, but they
still need to be checked manually with shapely.intersects(a, b) afterwards!**

More information here:
[https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query](https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query)

* **Return type:**
  `List`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]

Args:
: geo (shapely.Geometry): The geometry to query with
  predicate (Optional[ Literal[ &quot;intersects&quot;, &quot;within&quot;,
  <br/>
  > &quot;contains&quot;, &quot;overlaps&quot;, &quot;crosses&quot;,
  > &quot;touches&quot;, &quot;covers&quot;, &quot;covered_by&quot;,
  > &quot;contains_properly&quot;, &quot;dwithin&quot;, ] ], optional):
  > Which interaction to filter for. Defaults to None.
  <br/>
  distance (Optional[float], optional):
  : Must only be set for the &quot;dwithin&quot; predicate
    and controls its distance. Defaults to None.

Returns:
: List[ShapelyEntity]: The List of queried entities in the tree

<a id="mapping_common.map.MapTree.query_nearest"></a>

#### query_nearest(geo, max_distance=None, exclusive=False, all_matches=True)

Queries the distance from *geo* to its nearest entities in the tree

More information here:
[https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query_nearest](https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query_nearest)

* **Return type:**
  `List`[`Tuple`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity), `float`]]

Args:
: geo (shapely.Geometry): The geometry to query with
  max_distance (Optional[float], optional): Maximum distance for the query.
  <br/>
  > Defaults to None.
  <br/>
  exclusive (bool, optional): If True, ignores entities
  with a shape equal to geo. Defaults to False.
  all_matches (bool, optional): If True, all equidistant and intersected
  geometries will be returned. If False only the nearest. Defaults to True.

Returns:
: List[Tuple[ShapelyEntity, float]]: A List of Tuples.
  Each contains a queried entity and its distance to geo

<a id="mapping_common.map.MapTree.query_self"></a>

#### query_self(predicate=None, distance=None)

Queries interactions between the shapes inside this tree.

Removes any self intersections and duplicate interaction pairs.

* **Return type:**
  `List`[`Tuple`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity), [`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]]

Args:
: predicate (Optional[ Literal[ &quot;intersects&quot;, &quot;within&quot;,
  : &quot;contains&quot;, &quot;overlaps&quot;, &quot;crosses&quot;,
    &quot;touches&quot;, &quot;covers&quot;, &quot;covered_by&quot;,
    &quot;contains_properly&quot;, &quot;dwithin&quot;, ] ], optional):
    Which interaction to filter for. Defaults to None.
  <br/>
  distance (Optional[float], optional):
  : Must only be set for the &quot;dwithin&quot; predicate
    and controls its distance. Defaults to None.

Returns:
: List[Tuple[ShapelyEntity, ShapelyEntity]]:
  : Tuples of interacting entity pairs

<a id="mapping_common.map.MapTree.get_entity_in_front_or_back"></a>

#### get_entity_in_front_or_back(in_front=True)

Returns the first entity in front or back based on in_front

Projects a polygon to simulate the road
:rtype: `Optional`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]

Calculates the nearest entity on that polygon
Returns:

> Optional[Entity]: Entity in front

This could be extended with a curved polygon
if curved roads become a problem in the future

<a id="mapping_common.map.MapTree.is_lane_free"></a>

#### is_lane_free(right_lane=False, lane_length=20.0, lane_transform=0.0, reduce_lane=1.5, check_method='rectangle', min_coverage_percent=0.0, min_coverage_area=0.0, lane_angle=5.0, motion_aware=True)

* **Return type:**
  `Tuple`[[`LaneFreeState`](#mapping_common.map.LaneFreeState), `Optional`[`Geometry`]]

Returns if a lane left or right of our car is free.
There are three check methods available:

> - rectangle: checks if the lane is free by using a checkbox with size

> and position according to inputs
> - lanemarking: checks if the lane is free by using a checkbox that is
> placed between two lane markings
> - fallback: uses lanemarking as default and falls back to rectangle if
> lanemarking is not plausible

Parameters:
- right_lane (bool): If true, checks the right lane instead of the left lane
- lane_length (float): Sets the lane length that should be checked, in meters.

> Default value is 20 meters.
- lane_transform (float): Transforms the checked lane box to the front (>0) or
  back (<0) of the car, in meters. Default is 0 meter so the lane box originates
  > from the car position -> same distance to the front and rear get checked
- reduce_lane (float): Reduces the lane width that should be checked, in meters.
  : Default value is 1.5 meters.
- check_method (str): The method to check if the lane is free.
- lane_angle (float, optional): sets how many degrees the lanes may be skewed
  : in relation to each other that the check get executed. Defaults to 5.0 °,
    only used for lanemarking method.
- min_coverage_percent (float, optional): how much an entity must collide
  : with the checkbox in percent. Defaults to 0.0.
- min_coverage_area (float, optional): how much an entity must collide
  : with the checkbox in m2. Defaults to 0.0.
- motion_aware (bool, optional): if true, the lane check will be aware of
  : the motion of the entities. Defaults to True.

Default is “rectangle”.
Returns:

> Tuple[LaneFreeState, Optional[shapely.Geometry]]:
> return LaneFreeState and if available the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_rectangle"></a>

#### is_lane_free_rectangle(right_lane=False, lane_length=20.0, lane_transform=0.0, reduce_lane=1.5)

checks if the lane is free by using a checkbox with size and position
according to inputs

* **Return type:**
  `Tuple`[[`LaneFreeState`](#mapping_common.map.LaneFreeState), `Optional`[`Geometry`]]

Args:
: right_lane (bool, optional): if true checks for free lane on the right side.
  Defaults to False.
  lane_length (float, optional): length of the checkbox. Defaults to 20.0.
  lane_transform (float, optional): offset in x direction. Defaults to 0.0.
  reduce_lane (float, optional): impacts the width of checkbox
  (= width - reduce_lane). Defaults to 1.5.

Returns:
: Tuple[LaneFreeState, Optional[shapely.Geometry]]:
  return LaneFreeState and if available the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_lanemarking"></a>

#### is_lane_free_lanemarking(right_lane=False, lane_length=20.0, lane_transform=0.0, reduce_lane=1.5, lane_angle=5.0)

checks if a lane is free by using a checkbox that is placed between two lane
markings. The lane is considered free if there are no colliding entities with
the checkbox.

* **Return type:**
  `Tuple`[[`LaneFreeState`](#mapping_common.map.LaneFreeState), `Optional`[`Geometry`]]

Args:
: right_lane (bool, optional): if true checks for free lane on the right side.
  Defaults to False.
  lane_length (float, optional): length of the checkbox. Defaults to 20.0.
  lane_transform (float, optional): offset in x direction. Defaults to 0.0.
  reduce_lane (float, optional): impacts the width of checkbox
  (= width - reduce_lane). Defaults to 1.5.
  lane_angle (float, optional): sets how many degrees the lanes may be skewed
  in relation to each other that the check get executed. Defaults to 5.0 °

Returns:
: Tuple[LaneFreeState, Optional[shapely.Geometry]]: return if lane is free
  and the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_intersection"></a>

#### is_lane_free_intersection(lane_length=20.0, lane_transform_x=0.0)

Returns True if the opposing lane of our car is free.
Checks if a Polygon lane box intersects with any
relevant entities.

This is only meant to be used in intersections. Ignores entities
that are not moving towards the hero.

* **Return type:**
  `Tuple`[`bool`, `Optional`[`Polygon`]]

Parameters:
- lane_length (float): Sets the lane length that should be checked, in meters.

> Default value is 20 meters.
- lane_transform_x (float): Transforms the checked lane box to the front (>0) or
  back (<0) of the car, in meters. Default is 0 meter so the lane box originates
  > from the car position -> same distance to the front and rear get checked

Returns:
: (bool, [shapely.Polygon]): lane is free / not free,
  : collision masks used for the check

<a id="mapping_common.map.MapTree.get_nearest_entity"></a>

#### get_nearest_entity(mask, reference, min_coverage_percent=0.0, min_coverage_area=0.0)

Returns the nearest entity to *reference* that have
at least coverage % or area in the mask geometry.

* **Return type:**
  `Optional`[`Tuple`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity), `float`]]

Args:
: mask (shapely.Geometry): A Shapely Geometry object representing
  : the target area.
  <br/>
  reference (ShapelyEntity): Entity for the nearest distance calculation
  min_coverage_percent (float, optional):
  <br/>
  > How much of an entity has to be inside the collision mask in percent.
  > Defaults to 0.0.
  <br/>
  min_coverage_area (float, optional):
  : How much of an entity has to be inside the collision mask in m2.
    Defaults to 0.0.

Returns:
: Optional[Tuple[ShapelyEntity, float]]:
  : A Tuple of (Nearest Entity, Distance to reference)

<a id="mapping_common.map.MapTree.get_overlapping_entities"></a>

#### get_overlapping_entities(mask, min_coverage_percent=0.0, min_coverage_area=0.0)

Returns a list of entities that have at least coverage % or area in the
mask geometry.

* **Return type:**
  `List`[[`ShapelyEntity`](mapping_common.entity.md#mapping_common.entity.ShapelyEntity)]

Args:
: mask (shapely.Geometry): A Shapely Geometry object representing
  : the target area.
  <br/>
  min_coverage_percent (float, optional):
  : How much of an entity has to be inside the collision mask in percent.
    Defaults to 0.0.
  <br/>
  min_coverage_area (float, optional):
  : How much of an entity has to be inside the collision mask in m2.
    Defaults to 0.0.

Returns:
: List[ShapelyEntity]: A list of entities that have at least
  : coverage % or area in the polygon

<a id="mapping_common.map.build_global_hero_transform"></a>

### mapping_common.map.build_global_hero_transform(x, y, heading)

Builds a Transform2D representing the global position of the hero
based on its coordinates and heading

* **Return type:**
  [`Transform2D`](mapping_common.transform.md#mapping_common.transform.Transform2D)

Args:
: x (float): Global hero x coordinate
  y (float): Global hero y coordinate
  heading (float): hero heading

Returns:
: Transform2D: Global hero Transform2D

<a id="mapping_common.map.lane_free_filter"></a>

### mapping_common.map.lane_free_filter()

Creates the default flag filter for the lane free check

* **Return type:**
  [`FlagFilter`](mapping_common.entity.md#mapping_common.entity.FlagFilter)

Returns:
: FlagFilter
