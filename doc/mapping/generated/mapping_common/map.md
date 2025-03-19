<!-- markdownlint-disable -->
# Map documentation

## Table of Contents

* [mapping\_common.map](#mapping_common.map)
  * [Map](#mapping_common.map.Map)
    * [timestamp](#mapping_common.map.Map.timestamp)
    * [entities](#mapping_common.map.Map.entities)
    * [hero](#mapping_common.map.Map.hero)
    * [entities\_without\_hero](#mapping_common.map.Map.entities_without_hero)
    * [get\_lane\_y\_axis\_intersections](#mapping_common.map.Map.get_lane_y_axis_intersections)
    * [build\_tree](#mapping_common.map.Map.build_tree)
    * [filtered](#mapping_common.map.Map.filtered)
    * [to\_multi\_poly\_array](#mapping_common.map.Map.to_multi_poly_array)
  * [MapTree](#mapping_common.map.MapTree)
    * [filtered\_entities](#mapping_common.map.MapTree.filtered_entities)
    * [map](#mapping_common.map.MapTree.map)
    * [\_\_init\_\_](#mapping_common.map.MapTree.__init__)
    * [nearest](#mapping_common.map.MapTree.nearest)
    * [query](#mapping_common.map.MapTree.query)
    * [query\_nearest](#mapping_common.map.MapTree.query_nearest)
    * [query\_self](#mapping_common.map.MapTree.query_self)
    * [get\_entity\_in\_front\_or\_back](#mapping_common.map.MapTree.get_entity_in_front_or_back)
    * [is\_lane\_free](#mapping_common.map.MapTree.is_lane_free)
    * [is\_lane\_free\_rectangle](#mapping_common.map.MapTree.is_lane_free_rectangle)
    * [is\_lane\_free\_lanemarking](#mapping_common.map.MapTree.is_lane_free_lanemarking)
    * [is\_lane\_free\_intersection](#mapping_common.map.MapTree.is_lane_free_intersection)
    * [get\_nearest\_entity](#mapping_common.map.MapTree.get_nearest_entity)
    * [get\_overlapping\_entities](#mapping_common.map.MapTree.get_overlapping_entities)
  * [build\_global\_hero\_transform](#mapping_common.map.build_global_hero_transform)
  * [lane\_free\_filter](#mapping_common.map.lane_free_filter)

<a id="mapping_common.map"></a>

# mapping\_common.map

Contains map-related functions

**[API documentation](/doc/mapping/generated/mapping_common/map.md)**

<a id="mapping_common.map.Map"></a>

## Map

```python
@dataclass
class Map()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L47)

2 dimensional map for the intermediate layer

General information:
- 2D top-down map. The height(z) dimension is mostly useless
  for collision detection and path planning
- The map is based on the local car position and is only built using sensor data.
  No global positioning via GPS or similar is used.
- The map (0/0) is the center of the hero car
- All transformations to entities are relative to
  the hero car's coordinate system (position/heading)
- The map's x-axis is aligned with the heading of the hero car
- The map's y-axis points to the left of the hero car
- Coordinate system is a right-hand system like tf2 (can be visualized in RViz)
- The map might include the hero car as the first entity in entities

<a id="mapping_common.map.Map.timestamp"></a>

#### timestamp: `Time`

The timestamp this map was created at.

Should be the time when this map was initially sent off
from the mapping_data_integration node.

This timestamp is also the "freshest" compared to
the timestamps of all entities included in the map

<a id="mapping_common.map.Map.entities"></a>

#### entities: `List[Entity]`

The entities this map consists out of

Note that this list might also include the hero car (as first element of this list)

<a id="mapping_common.map.Map.hero"></a>

#### hero

```python
def hero() -> Optional[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L79)

Returns the entity of the hero car if it is the first element of the map

**Returns**:

- `Optional[Entity]` - Entity of the hero car

<a id="mapping_common.map.Map.entities_without_hero"></a>

#### entities\_without\_hero

```python
def entities_without_hero() -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L92)

Returns the entities without the hero car

Only checks if the first entity is_hero

**Returns**:

- `List[Entity]` - Entities without the hero car

<a id="mapping_common.map.Map.get_lane_y_axis_intersections"></a>

#### get\_lane\_y\_axis\_intersections

```python
def get_lane_y_axis_intersections(direction: str = "left") -> dict
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L104)

calculates the intersections of the lanemarks in lane_pos direction

**Arguments**:

- `direction` _str_ - lanemarks on "left", "right" or "both" will be checked.
  Other inputs will be ignored
  

**Returns**:

  dict{uuid, coordinate}: dictionary with uuid of lanemark as keys
  and coordinates as according entries

<a id="mapping_common.map.Map.build_tree"></a>

#### build\_tree

```python
def build_tree(
        f: Optional[FlagFilter] = None,
        filter_fn: Optional[Callable[[Entity], bool]] = None) -> "MapTree"
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L138)

Creates a filtered MapTree

**IMPORTANT**: The map
MUST NOT BE MODIFIED WHILE USING THE TREE,
otherwise results will be invalid or crash

Useful for for quickly calculating which entities of a map are
the nearest or (intersect, touch, etc.) with a given geometry

**Arguments**:

- `f` _Optional[FlagFilter], optional_ - Filtering with FlagFilter.
  Defaults to None.
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  Filtering with function/lambda. Defaults to None.
  

**Returns**:

  MapTree

<a id="mapping_common.map.Map.filtered"></a>

#### filtered

```python
def filtered(
        f: Optional[FlagFilter] = None,
        filter_fn: Optional[Callable[[Entity], bool]] = None) -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L163)

Filters self.entities

**Arguments**:

- `f` _Optional[FlagFilter], optional_ - Filtering with FlagFilter.
  Defaults to None.
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  Filtering with function/lambda. Defaults to None.
  

**Returns**:

- `List[Entity]` - List of entities both filters were matching for

<a id="mapping_common.map.Map.to_multi_poly_array"></a>

#### to\_multi\_poly\_array

```python
def to_multi_poly_array(area_to_incorporate: Tuple[int, int],
                        resolution_scale: int) -> npt.NDArray
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L181)

Takes the entities without hero of the map and draws the contours onto
a numpy array

**Arguments**:

- `area_to_incorporate` _Tuple[int, int]_ - the area, in which the entities
  have to be plotted
- `resolution_scale` _int_ - since the array has only integer indices,
  scale the array
  

**Returns**:

- `npt.NDArray` - array with contours drawn in

<a id="mapping_common.map.MapTree"></a>

## MapTree

```python
@dataclass(init=False)
class MapTree()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L245)

An acceleration structure around the shapely.STRtree

**IMPORTANT**: The map this tree was created with
MUST NOT BE MODIFIED WHILE USING THE TREE,
otherwise results will be invalid or crash

Useful for for quickly calculating which entities of a map are
the nearest or (intersect, touch, etc.) with a given geometry

The map's entities can optionally be filtered upon tree creation

<a id="mapping_common.map.MapTree.filtered_entities"></a>

#### filtered\_entities: `List[ShapelyEntity]`

Only the entities of this tree that weren't filtered out from the map.

Also includes their shapely.Polygon

<a id="mapping_common.map.MapTree.map"></a>

#### map: `Map`

The unfiltered map this tree was created with

<a id="mapping_common.map.MapTree.__init__"></a>

#### \_\_init\_\_

```python
def __init__(map: Map,
             f: Optional[FlagFilter] = None,
             filter_fn: Optional[Callable[[Entity], bool]] = None)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L269)

Creates a shapely.STRtree based on the given map and filtering

Both filtering methods can be combined.
Both filters need to match for the entity to match.

**Arguments**:

  map (Map)
- `f` _Optional[FlagFilter], optional_ - Filtering with FlagFilter.
  Defaults to None.
  filter_fn (Optional[Callable[[Entity], bool]], optional):
  Filtering with function/lambda. Defaults to None.

<a id="mapping_common.map.MapTree.nearest"></a>

#### nearest

```python
def nearest(geo: shapely.Geometry) -> Optional[ShapelyEntity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L295)

Returns the nearest Entity inside the tree based on geo

More information here:
https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.nearest

**Arguments**:

- `geo` _shapely.Geometry_ - Geometry to calculate the nearest entity to
  

**Returns**:

- `Optional[ShapelyEntity]` - If the tree is empty, will return None.
  Otherwise will return the nearest Entity

<a id="mapping_common.map.MapTree.query"></a>

#### query

```python
def query(geo: shapely.Geometry,
          predicate: Optional[Literal[
              "intersects",
              "within",
              "contains",
              "overlaps",
              "crosses",
              "touches",
              "covers",
              "covered_by",
              "contains_properly",
              "dwithin",
          ]] = None,
          distance: Optional[float] = None) -> List[ShapelyEntity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L313)

Calculates which entities interact with *geo*

**IMPORTANT: The query might include false positives,
because the STRtree seems to only roughly check for intersections.
It includes all entities that MIGHT intersect with geo, but they
still need to be checked manually with shapely.intersects(a, b) afterwards!**

More information here:
https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query

**Arguments**:

- `geo` _shapely.Geometry_ - The geometry to query with
  predicate (Optional[ Literal[ &quot;intersects&quot;, &quot;within&quot;,
  &quot;contains&quot;, &quot;overlaps&quot;, &quot;crosses&quot;,
  &quot;touches&quot;, &quot;covers&quot;, &quot;covered_by&quot;,
  &quot;contains_properly&quot;, &quot;dwithin&quot;, ] ], optional):
  Which interaction to filter for. Defaults to None.
  distance (Optional[float], optional):
  Must only be set for the &quot;dwithin&quot; predicate
  and controls its distance. Defaults to None.
  

**Returns**:

- `List[ShapelyEntity]` - The List of queried entities in the tree

<a id="mapping_common.map.MapTree.query_nearest"></a>

#### query\_nearest

```python
def query_nearest(
        geo: shapely.Geometry,
        max_distance: Optional[float] = None,
        exclusive: bool = False,
        all_matches: bool = True) -> List[Tuple[ShapelyEntity, float]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L361)

Queries the distance from *geo* to its nearest entities in the tree

More information here:
https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree.query_nearest

**Arguments**:

- `geo` _shapely.Geometry_ - The geometry to query with
- `max_distance` _Optional[float], optional_ - Maximum distance for the query.
  Defaults to None.
- `exclusive` _bool, optional_ - If True, ignores entities
  with a shape equal to geo. Defaults to False.
- `all_matches` _bool, optional_ - If True, all equidistant and intersected
  geometries will be returned. If False only the nearest. Defaults to True.
  

**Returns**:

  List[Tuple[ShapelyEntity, float]]: A List of Tuples.
  Each contains a queried entity and its distance to geo

<a id="mapping_common.map.MapTree.query_self"></a>

#### query\_self

```python
def query_self(
    predicate: Optional[Literal[
        "intersects",
        "within",
        "contains",
        "overlaps",
        "crosses",
        "touches",
        "covers",
        "covered_by",
        "contains_properly",
        "dwithin",
    ]] = None,
    distance: Optional[float] = None
) -> List[Tuple[ShapelyEntity, ShapelyEntity]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L400)

Queries interactions between the shapes inside this tree.

Removes any self intersections and duplicate interaction pairs.

**Arguments**:

  predicate (Optional[ Literal[ &quot;intersects&quot;, &quot;within&quot;,
  &quot;contains&quot;, &quot;overlaps&quot;, &quot;crosses&quot;,
  &quot;touches&quot;, &quot;covers&quot;, &quot;covered_by&quot;,
  &quot;contains_properly&quot;, &quot;dwithin&quot;, ] ], optional):
  Which interaction to filter for. Defaults to None.
  distance (Optional[float], optional):
  Must only be set for the &quot;dwithin&quot; predicate
  and controls its distance. Defaults to None.
  

**Returns**:

  List[Tuple[ShapelyEntity, ShapelyEntity]]:
  Tuples of interacting entity pairs

<a id="mapping_common.map.MapTree.get_entity_in_front_or_back"></a>

#### get\_entity\_in\_front\_or\_back

```python
def get_entity_in_front_or_back(in_front=True) -> Optional[ShapelyEntity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L453)

Returns the first entity in front or back based on in_front

Projects a polygon to simulate the road
Calculates the nearest entity on that polygon

**Returns**:

- `Optional[Entity]` - Entity in front
  This could be extended with a curved polygon
  if curved roads become a problem in the future

<a id="mapping_common.map.MapTree.is_lane_free"></a>

#### is\_lane\_free

```python
def is_lane_free(
    right_lane: bool = False,
    lane_length: float = 20.0,
    lane_transform: float = 0.0,
    reduce_lane: float = 1.5,
    check_method: Literal[
        "rectangle",
        "lanemarking",
        "fallback",
        # "trajectory" not implemented yet
    ] = "rectangle",
    min_coverage_percent: float = 0.0,
    min_coverage_area: float = 0.0,
    lane_angle: float = 5.0,
    motion_aware: bool = True
) -> Tuple[LaneFreeState, Optional[shapely.Geometry]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L482)

Returns if a lane left or right of our car is free.
There are three check methods available:
- rectangle: checks if the lane is free by using a checkbox with size
and position according to inputs
- lanemarking: checks if the lane is free by using a checkbox that is
placed between two lane markings
- fallback: uses lanemarking as default and falls back to rectangle if
lanemarking is not plausible

**Arguments**:

  - right_lane (bool): If true, checks the right lane instead of the left lane
  - lane_length (float): Sets the lane length that should be checked, in meters.
  Default value is 20 meters.
  - lane_transform (float): Transforms the checked lane box to the front (>0) or
  back (<0) of the car, in meters. Default is 0 meter so the lane box originates
  from the car position -> same distance to the front and rear get checked
  - reduce_lane (float): Reduces the lane width that should be checked, in meters.
  Default value is 1.5 meters.
  - check_method (str): The method to check if the lane is free.
  - lane_angle (float, optional): sets how many degrees the lanes may be skewed
  in relation to each other that the check get executed. Defaults to 5.0 °,
  only used for lanemarking method.
  - min_coverage_percent (float, optional): how much an entity must collide
  with the checkbox in percent. Defaults to 0.0.
  - min_coverage_area (float, optional): how much an entity must collide
  with the checkbox in m2. Defaults to 0.0.
  - motion_aware (bool, optional): if true, the lane check will be aware of
  the motion of the entities. Defaults to True.
  
  Default is "rectangle".

**Returns**:

  Tuple[LaneFreeState, Optional[shapely.Geometry]]:
  return LaneFreeState and if available the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_rectangle"></a>

#### is\_lane\_free\_rectangle

```python
def is_lane_free_rectangle(
    right_lane: bool = False,
    lane_length: float = 20.0,
    lane_transform: float = 0.0,
    reduce_lane: float = 1.5
) -> Tuple[LaneFreeState, Optional[shapely.Geometry]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L599)

checks if the lane is free by using a checkbox with size and position
according to inputs

**Arguments**:

- `right_lane` _bool, optional_ - if true checks for free lane on the right side.
  Defaults to False.
- `lane_length` _float, optional_ - length of the checkbox. Defaults to 20.0.
- `lane_transform` _float, optional_ - offset in x direction. Defaults to 0.0.
- `reduce_lane` _float, optional_ - impacts the width of checkbox
  (= width - reduce_lane). Defaults to 1.5.
  

**Returns**:

  Tuple[LaneFreeState, Optional[shapely.Geometry]]:
  return LaneFreeState and if available the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_lanemarking"></a>

#### is\_lane\_free\_lanemarking

```python
def is_lane_free_lanemarking(
    right_lane: bool = False,
    lane_length: float = 20.0,
    lane_transform: float = 0.0,
    reduce_lane: float = 1.5,
    lane_angle: float = 5.0
) -> Tuple[LaneFreeState, Optional[shapely.Geometry]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L646)

checks if a lane is free by using a checkbox that is placed between two lane
markings. The lane is considered free if there are no colliding entities with
the checkbox.

**Arguments**:

- `right_lane` _bool, optional_ - if true checks for free lane on the right side.
  Defaults to False.
- `lane_length` _float, optional_ - length of the checkbox. Defaults to 20.0.
- `lane_transform` _float, optional_ - offset in x direction. Defaults to 0.0.
- `reduce_lane` _float, optional_ - impacts the width of checkbox
  (= width - reduce_lane). Defaults to 1.5.
- `lane_angle` _float, optional_ - sets how many degrees the lanes may be skewed
  in relation to each other that the check get executed. Defaults to 5.0 °
  

**Returns**:

  Tuple[LaneFreeState, Optional[shapely.Geometry]]: return if lane is free
  and the checkbox shape

<a id="mapping_common.map.MapTree.is_lane_free_intersection"></a>

#### is\_lane\_free\_intersection

```python
def is_lane_free_intersection(
        lane_length: float = 20.0,
        lane_transform_x: float = 0.0
) -> Tuple[bool, Optional[shapely.Polygon]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L730)

Returns True if the opposing lane of our car is free.
Checks if a Polygon lane box intersects with any
relevant entities.

This is only meant to be used in intersections. Ignores entities
that are not moving towards the hero.

**Arguments**:

  - lane_length (float): Sets the lane length that should be checked, in meters.
  Default value is 20 meters.
  - lane_transform_x (float): Transforms the checked lane box to the front (>0) or
  back (<0) of the car, in meters. Default is 0 meter so the lane box originates
  from the car position -> same distance to the front and rear get checked

**Returns**:

  (bool, [shapely.Polygon]): lane is free / not free,
  collision masks used for the check

<a id="mapping_common.map.MapTree.get_nearest_entity"></a>

#### get\_nearest\_entity

```python
def get_nearest_entity(
        mask: shapely.Geometry,
        reference: ShapelyEntity,
        min_coverage_percent: float = 0.0,
        min_coverage_area: float = 0.0
) -> Optional[Tuple[ShapelyEntity, float]]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L787)

Returns the nearest entity to *reference* that have
at least coverage % or area in the mask geometry.

**Arguments**:

- `mask` _shapely.Geometry_ - A Shapely Geometry object representing
  the target area.
- `reference` _ShapelyEntity_ - Entity for the nearest distance calculation
  min_coverage_percent (float, optional):
  How much of an entity has to be inside the collision mask in percent.
  Defaults to 0.0.
  min_coverage_area (float, optional):
  How much of an entity has to be inside the collision mask in m2.
  Defaults to 0.0.
  

**Returns**:

  Optional[Tuple[ShapelyEntity, float]]:
  A Tuple of (Nearest Entity, Distance to reference)

<a id="mapping_common.map.MapTree.get_overlapping_entities"></a>

#### get\_overlapping\_entities

```python
def get_overlapping_entities(
        mask: shapely.Geometry,
        min_coverage_percent: float = 0.0,
        min_coverage_area: float = 0.0) -> List[ShapelyEntity]
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L829)

Returns a list of entities that have at least coverage % or area in the
mask geometry.

**Arguments**:

- `mask` _shapely.Geometry_ - A Shapely Geometry object representing
  the target area.
  min_coverage_percent (float, optional):
  How much of an entity has to be inside the collision mask in percent.
  Defaults to 0.0.
  min_coverage_area (float, optional):
  How much of an entity has to be inside the collision mask in m2.
  Defaults to 0.0.
  

**Returns**:

- `List[ShapelyEntity]` - A list of entities that have at least
  coverage % or area in the polygon

<a id="mapping_common.map.build_global_hero_transform"></a>

#### build\_global\_hero\_transform

```python
def build_global_hero_transform(x: float, y: float,
                                heading: float) -> Transform2D
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L895)

Builds a Transform2D representing the global position of the hero
based on its coordinates and heading

**Arguments**:

- `x` _float_ - Global hero x coordinate
- `y` _float_ - Global hero y coordinate
- `heading` _float_ - hero heading
  

**Returns**:

- `Transform2D` - Global hero Transform2D

<a id="mapping_common.map.lane_free_filter"></a>

#### lane\_free\_filter

```python
def lane_free_filter() -> FlagFilter
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/map.py#L911)

Creates the default flag filter for the lane free check

**Returns**:

  FlagFilter

