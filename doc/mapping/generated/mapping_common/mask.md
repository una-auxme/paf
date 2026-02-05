<!-- markdownlint-disable -->
# Mask documentation

## Table of Contents

* [mapping\_common.mask](#mapping_common.mask)
  * [curve\_to\_polygon](#mapping_common.mask.curve_to_polygon)
  * [split\_line\_at](#mapping_common.mask.split_line_at)
  * [clamp\_line](#mapping_common.mask.clamp_line)
  * [ros\_path\_to\_line](#mapping_common.mask.ros_path_to_line)
  * [line\_to\_ros\_path](#mapping_common.mask.line_to_ros_path)
  * [build\_trajectory](#mapping_common.mask.build_trajectory)
  * [build\_trajectory\_shape](#mapping_common.mask.build_trajectory_shape)
  * [project\_plane](#mapping_common.mask.project_plane)
  * [build\_lead\_vehicle\_collision\_masks](#mapping_common.mask.build_lead_vehicle_collision_masks)
  * [build\_trajectory\_from\_start](#mapping_common.mask.build_trajectory_from_start)
  * [point\_along\_line\_angle](#mapping_common.mask.point_along_line_angle)
  * [create\_lane\_box](#mapping_common.mask.create_lane_box)

<a id="mapping_common.mask"></a>

# mapping\_common.mask

Collection of functions to create shapely Geometry

**[API documentation](/doc/mapping/generated/mapping_common/mask.md)**

Overview of the main components:
- Trajectory functions:
  - Convert ROS Path to shapely.LineString and back: ros_path_to_line, line_to_ros_path:
  - Accurate line splitting: split_line_at, clamp_line
  - Trajectory generation functions: build_trajectory, build_trajectory_from_start
- Collision mask generation:
  - These masks can be used together with MapTree.get_overlapping_entities().
  - For lead vehicle detection: project_plane, build_trajectory_shape,
    build_lead_vehicle_collision_masks
- Some lane mask functions (Main parts of lane mask generation are located in MapTree)

<a id="mapping_common.mask.curve_to_polygon"></a>

#### curve\_to\_polygon

```python
def curve_to_polygon(line: shapely.LineString,
                     width: float) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L32)

Creates a polygon with a specified width around a given line.

**Arguments**:

- `line` _shapely.LineString_ - line
- `width` _float_ - Width of the result
  

**Returns**:

  shapely.Polygon

<a id="mapping_common.mask.split_line_at"></a>

#### split\_line\_at

```python
def split_line_at(
        line: shapely.LineString,
        distance: float) -> Tuple[Optional[LineString], Optional[LineString]]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L59)

Splits line at the given distance from the line start

**Arguments**:

- `line` _shapely.LineString_ - Line to split
- `distance` _float_ - length from the start
  

**Returns**:

  Tuple[Optional[LineString], Optional[LineString]]: (before, after) Tuple:
  Line before and after the split.
  Either of them might be None depending on the split position.

<a id="mapping_common.mask.clamp_line"></a>

#### clamp\_line

```python
def clamp_line(
        line: shapely.LineString,
        start_distance: float = 0.0,
        end_distance: Optional[float] = None) -> Optional[shapely.LineString]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L140)

Clamps line based on the two distances from the line start

**Arguments**:

- `line` _shapely.LineString_ - Line to clamp
- `start_distance` _float, optional_ - Distance for the first cut
  from the line start point. Defaults to 0.0.
- `end_distance` _Optional[float], optional_ - Distance for the second cut
  from the original line start point.
  If None only the starting section is cut off. Defaults to None.
  

**Returns**:

- `Optional[shapely.LineString]` - None if the clamping leaves nothing

<a id="mapping_common.mask.ros_path_to_line"></a>

#### ros\_path\_to\_line

```python
def ros_path_to_line(path: NavPath,
                     start_idx: int = 0,
                     end_idx: Optional[int] = None) -> shapely.LineString
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L171)

Converts a ROS path into a shapely line

**Arguments**:

- `path` _NavPath_ - ROS nav_msgs.msg.Path
- `start_idx` _int, optional_ - Points before the start index will
  be left out. Defaults to 0.
- `end_idx` _Optional[int], optional_ - Points after(including the)
  end index will be left out. Defaults to None.
  

**Returns**:

- `shapely.LineString` - line

<a id="mapping_common.mask.line_to_ros_path"></a>

#### line\_to\_ros\_path

```python
def line_to_ros_path(line: shapely.LineString) -> NavPath
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L196)

Converts a shapely line into a ROS path

**Arguments**:

- `line` _shapely.LineString_ - line
  

**Returns**:

- `Path` - ROS nav_msgs.msg.Path

<a id="mapping_common.mask.build_trajectory"></a>

#### build\_trajectory

```python
def build_trajectory(global_trajectory: Union[NavPath, shapely.LineString],
                     global_hero_transform: Transform2D,
                     max_length: Optional[float] = None,
                     current_wp_idx: int = 0,
                     max_wp_count: Optional[int] = None,
                     centered: bool = False) -> Optional[shapely.LineString]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L213)

Builds a local trajectory line based on the global trajectory
the and global_hero_transform and returns it as line

The returned line starts at the nearest position (of the trajectory)
to the global_hero_transform

The global_hero_transform can be built with
mapping_common.map.build_global_hero_transform().

When centered is True, the trajectory start point will be centered onto (0, 0).
Centered mode must not be used for navigating,
but can be used for collision avoidance / acc.

**Arguments**:

- `global_trajectory` _NavPath_ - NavPath trajectory in global coordinates
- `global_hero_transform` _Transform2D_ - Global Transform2D of the hero
- `max_length` _Optional[float], optional_ - Maximum length of the resulting line.
  Defaults to None.
- `current_wp_idx` _int, optional_ - Waypoint index for
  very rough clamping of the NavPath. Defaults to 0.
- `max_wp_count` _Optional[int], optional_ - Max waypoint count of the NavPath
  for very rough clamping of the NavPath. Defaults to None.
- `centered` _bool, optional_ - Centered mode. Defaults to False.
  

**Returns**:

- `Optional[shapely.LineString]` - Local line based on the trajectory

<a id="mapping_common.mask.build_trajectory_shape"></a>

#### build\_trajectory\_shape

```python
def build_trajectory_shape(
        global_trajectory: NavPath,
        global_hero_transform: Transform2D,
        width: float = 1.0,
        start_dist_from_hero: Optional[float] = 0.0,
        max_length: Optional[float] = None,
        current_wp_idx: int = 0,
        max_wp_count: Optional[int] = None,
        centered: bool = False) -> Optional[shapely.Polygon]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L279)

Builds a local trajectory shape based on the global trajectory
the and global_hero_transform

The returned shape starts at the nearest position (of the trajectory)
to the global_hero_transform

**Arguments**:

- `global_trajectory` _NavPath_ - NavPath trajectory in global coordinates
- `global_hero_transform` _Transform2D_ - Global Transform2D of the hero
- `width` _float, optional_ - Width of the trajectory shape. Defaults to 1.0.
  start_dist_from_hero (Optional[float], optional):
  Removes the first meters from the trajectory. Defaults to 0.0.
  max_length (Optional[float], optional):
  Maximum length of the resulting shape. Defaults to None.
- `current_wp_idx` _int, optional_ - Waypoint index for
  very rough clamping of the NavPath. Defaults to 0.
- `max_wp_count` _Optional[int], optional_ - Max waypoint index of the NavPath
  for very rough clamping of the NavPath. Defaults to None.
- `centered` _bool, optional_ - Centered mode. Defaults to False.
  

**Returns**:

- `Optional[shapely.Polygon]` - Local shape based on the trajectory

<a id="mapping_common.mask.project_plane"></a>

#### project\_plane

```python
def project_plane(size_x: float,
                  size_y: float,
                  start_point: Optional[Point2] = None) -> shapely.Polygon
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L332)

Projects a rectangular plane starting from (0, 0) forward in the x-direction.

**Arguments**:

- `size_x` _float_ - Length of the plane along the x-axis.
- `size_y` _float_ - Width of the plane along the y-axis.
  start_point(Optional[Point2], optional):
  Start point in the low y-center of the rectangle.
- `Default` - Point2.zero()
  

**Returns**:

- `Polygon` - A Shapely Polygon representing the plane.

<a id="mapping_common.mask.build_lead_vehicle_collision_masks"></a>

#### build\_lead\_vehicle\_collision\_masks

```python
def build_lead_vehicle_collision_masks(
        start_point: Point2,
        width: float,
        trajectory_local: NavPath,
        front_mask_size: float,
        max_trajectory_check_length: Optional[float] = None,
        rotate_front_mask: float = 0.0,
        max_centering_dist: Optional[float] = 0.5) -> List[shapely.Polygon]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L364)

Builds a list of collision masks for determining the lead vehicle
in front of the hero.

**Arguments**:

- `start_point` _Point2_ - Start point of the collision masks.
  Should be the front center of the hero
- `width` _float_ - Width of the collision masks. Should match the width of the hero
- `trajectory_local` _NavPath_ - Planned local trajectory
- `front_mask_size` _float_ - Size of the static mask in front
  max_trajectory_check_length (Optional[float], optional):
  Max length of the collision masks. Defaults to None.
- `rotate_front_mask` _float, optional_ - rotates the static mask
  in front. Usecase: Adjust with the steering angle.
- `max_centering_dist` _Optional[float], optional_ - Centers the trajectory part of
  the mask onto the front mask, if they align closely enough.
  If None-> No centering.
  

**Returns**:

  List[shapely.Polygon]

<a id="mapping_common.mask.build_trajectory_from_start"></a>

#### build\_trajectory\_from\_start

```python
def build_trajectory_from_start(
    trajectory_local: NavPath,
    start_point: Point2,
    max_centering_dist: Optional[float] = None
) -> Optional[shapely.LineString]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L432)

Build a trajectory based on *trajectory_local* that starts at *start_point*

**Arguments**:

- `trajectory_local` _NavPath_ - _description_
- `start_point` _Point2_ - _description_
- `max_centering_dist` _Optional[float], optional_ - Centers *trajectory_local*
  onto *start_point* if they align closely enough.
  Defaults to None-> No centering.
  

**Returns**:

- `Optional[shapely.LineString]` - _description_

<a id="mapping_common.mask.point_along_line_angle"></a>

#### point\_along\_line\_angle

```python
def point_along_line_angle(x: float, y: float, angle: float,
                           distance: float) -> Point2
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L480)

Calculates a point along a straight line with a given angle and distance.

**Arguments**:

- `x` _float_ - x-coordinate of the original position
- `y` _float_ - y-coordinate of the original position
- `angle` _float_ - Angle of the straight line (in rad)
- `distance` _float_ - Distance along the straight line (positive or negative)

**Returns**:

- `Point2(x,y)` - x-y-coordinates of new point as Point2

<a id="mapping_common.mask.create_lane_box"></a>

#### create\_lane\_box

```python
def create_lane_box(y_axis_line: LineString, lane_close_hero: Entity,
                    lane_further_hero: Entity, lane_pos: int,
                    lane_length: float, lane_transform: float,
                    reduce_lane: float) -> shapely.Geometry
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/mask.py#L498)

helper function to create a lane box entity

**Arguments**:

- `y_axis_line` _LineString_ - check shape y-axis line
- `lane_close_hero` _Entity_ - the lane marking entity that is closer to the car
- `lane_further_hero` _Entity_ - the lane marking entity that is further away
  from the car
- `lane_pos` _int_ - to check if the lane is on the left or right side of the car
- `lane_length` _float_ - length of the lane box
- `lane_transform` _float_ - transform of the lane box
- `reduce_lane` _float_ - reduce the lane
  

**Returns**:

- `lane_box` _Geometry_ - created lane box shape

