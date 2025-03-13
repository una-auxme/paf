# mapping_common.mask module

- [mapping_common.mask module]()
  - [`curve_to_polygon()`](#mapping_common.mask.curve_to_polygon)
  - [`split_line_at()`](#mapping_common.mask.split_line_at)
  - [`clamp_line()`](#mapping_common.mask.clamp_line)
  - [`ros_path_to_line()`](#mapping_common.mask.ros_path_to_line)
  - [`line_to_ros_path()`](#mapping_common.mask.line_to_ros_path)
  - [`build_trajectory()`](#mapping_common.mask.build_trajectory)
  - [`build_trajectory_shape()`](#mapping_common.mask.build_trajectory_shape)
  - [`project_plane()`](#mapping_common.mask.project_plane)
  - [`build_lead_vehicle_collision_masks()`](#mapping_common.mask.build_lead_vehicle_collision_masks)
  - [`build_trajectory_from_start()`](#mapping_common.mask.build_trajectory_from_start)
  - [`point_along_line_angle()`](#mapping_common.mask.point_along_line_angle)
  - [`create_lane_box()`](#mapping_common.mask.create_lane_box)

<a id="mapping_common.mask.curve_to_polygon"></a>

### mapping_common.mask.curve_to_polygon(line, width)

Creates a polygon with a specified width around a given line.

* **Return type:**
  `Polygon`

Args:
: line (shapely.LineString)
  width (float): Width of the result

Returns:
: shapely.Polygon

<a id="mapping_common.mask.split_line_at"></a>

### mapping_common.mask.split_line_at(line, distance)

Splits line at the given distance from the line start

* **Return type:**
  `Tuple`[`Optional`[`LineString`], `Optional`[`LineString`]]

Args:
: line (shapely.LineString): Line to split
  distance (float)

Returns:
: Tuple[Optional[LineString], Optional[LineString]]:
  : (before, after) Tuple: Line before and after the split.
    Either of them might be None depending on the split position.

<a id="mapping_common.mask.clamp_line"></a>

### mapping_common.mask.clamp_line(line, start_distance=0.0, end_distance=None)

Clamps line based on the two distances from the line start

* **Return type:**
  `Optional`[`LineString`]

Args:
: line (shapely.LineString): Line to clamp
  start_distance (float, optional):
  <br/>
  > Distance for the first cut from the line start point. Defaults to 0.0.
  <br/>
  end_distance (Optional[float], optional):
  : Distance for the second cut from the original line start point.
    If None only the starting section is cut off. Defaults to None.

Returns:
: Optional[shapely.LineString]: None if the clamping leaves nothing

<a id="mapping_common.mask.ros_path_to_line"></a>

### mapping_common.mask.ros_path_to_line(path, start_idx=0, end_idx=None)

* **Return type:**
  `LineString`

<a id="mapping_common.mask.line_to_ros_path"></a>

### mapping_common.mask.line_to_ros_path(line)

* **Return type:**
  `Path`

<a id="mapping_common.mask.build_trajectory"></a>

### mapping_common.mask.build_trajectory(global_trajectory, global_hero_transform, max_length=None, current_wp_idx=0, max_wp_count=None, centered=False)

Builds a local trajectory line based on the global trajectory
the and global_hero_transform and returns it as line

The returned line starts at the nearest position (of the trajectory)
to the global_hero_transform

The global_hero_transform can be built with
mapping_common.map.build_global_hero_transform().

When centered is true, the trajectory start point will be centered onto (0, 0).
Centered mode must not be used for navigating,
but can be used for collision avoidance / acc.

* **Return type:**
  `Optional`[`LineString`]

Args:
: global_trajectory (NavPath): NavPath trajectory in global coordinates
  global_hero_transform (Transform2D): Global Transform2D of the hero
  max_length (Optional[float], optional): Maximum length of the resulting line.
  <br/>
  > Defaults to None.
  <br/>
  current_wp_idx (int, optional): Waypoint index for
  : very rough clamping of the NavPath. Defaults to 0.
  <br/>
  max_wp_count (Optional[int], optional): Max waypoint count of the NavPath
  : for very rough clamping of the NavPath. Defaults to None.
  <br/>
  centered (bool, optional): Centered mode. Defaults to False.

Returns:
: Optional[shapely.LineString]: Local line based on the trajectory

<a id="mapping_common.mask.build_trajectory_shape"></a>

### mapping_common.mask.build_trajectory_shape(global_trajectory, global_hero_transform, width=1.0, start_dist_from_hero=0.0, max_length=None, current_wp_idx=0, max_wp_count=None, centered=False)

Builds a local trajectory shape based on the global trajectory
the and global_hero_transform

The returned shape starts at the nearest position (of the trajectory)
to the global_hero_transform

* **Return type:**
  `Optional`[`Polygon`]

Args:
: global_trajectory (NavPath): NavPath trajectory in global coordinates
  global_hero_transform (Transform2D): Global Transform2D of the hero
  width (float, optional): Width of the trajectory shape. Defaults to 1.0.
  start_dist_from_hero (Optional[float], optional):
  <br/>
  > Removes the first meters from the trajectory. Defaults to 0.0.
  <br/>
  max_length (Optional[float], optional):
  : Maximum length of the resulting shape. Defaults to None.
  <br/>
  current_wp_idx (int, optional): Waypoint index for
  : very rough clamping of the NavPath. Defaults to 0.
  <br/>
  max_wp_count (Optional[int], optional): Max waypoint index of the NavPath
  : for very rough clamping of the NavPath. Defaults to None.
  <br/>
  centered (bool, optional): Centered mode. Defaults to False.

Returns:
: Optional[shapely.Polygon]: Local shape based on the trajectory

<a id="mapping_common.mask.project_plane"></a>

### mapping_common.mask.project_plane(size_x, size_y, start_point=None)

Projects a rectangular plane starting from (0, 0) forward in the x-direction.

Parameters:
- size_x (float): Length of the plane along the x-axis.
:rtype: `Polygon`

- size_y (float): Width of the plane along the y-axis.
- start_point(Optional[Point2], optional):
  : Start point in the low y-center of the rectangle.
    Default: Point2.zero()

Returns:
- Polygon: A Shapely Polygon representing the plane.

<a id="mapping_common.mask.build_lead_vehicle_collision_masks"></a>

### mapping_common.mask.build_lead_vehicle_collision_masks(start_point, width, trajectory_local, front_mask_size, max_trajectory_check_length=None, rotate_front_mask=0.0, max_centering_dist=0.5)

Builds a list of collision masks for determining the lead vehicle
in front of the hero.

* **Return type:**
  `List`[`Polygon`]

Args:
: start_point (Point2): Start point of the collision masks.
  : Should be the front center of the hero
  <br/>
  width (float): Width of the collision masks. Should match the width of the hero
  trajectory_local (NavPath): Planned local trajectory
  front_mask_size (float): Size of the static mask in front
  max_trajectory_check_length (Optional[float], optional):
  <br/>
  > Max length of the collision masks. Defaults to None.
  <br/>
  rotate_front_mask (float, optional): rotates the static mask in front.
  : Usecase: Adjust with the steering angle.
  <br/>
  max_centering_dist (Optional[float], optional):
  : Centers the trajectory part of the mask onto the front mask,
    if they align closely enough
    If None-> No centering.

Returns:
: List[shapely.Polygon]

<a id="mapping_common.mask.build_trajectory_from_start"></a>

### mapping_common.mask.build_trajectory_from_start(trajectory_local, start_point, max_centering_dist=None)

Build a trajectory based on *trajectory_local* that starts at *start_point*

* **Return type:**
  `Optional`[`LineString`]

Args:
: trajectory_local (NavPath): \_description_
  start_point (Point2): \_description_
  max_centering_dist (Optional[float], optional):
  <br/>
  > Centers *trajectory_local* onto *start_point* if they align closely enough.
  > Defaults to None-> No centering.

Returns:
: Optional[shapely.LineString]: \_description_

<a id="mapping_common.mask.point_along_line_angle"></a>

### mapping_common.mask.point_along_line_angle(x, y, angle, distance)

Calculates a point along a straight line with a given angle and distance.

Parameters:
- x (float): x-coordinate of the original position
- y (float): y-coordinate of the original position
- angle (float): Angle of the straight line (in rad)
:rtype: [`Point2`](mapping_common.transform.md#mapping_common.transform.Point2)

- distance (float): Distance along the straight line (positive or negative)

Returns:
: Point2(x,y): x-y-coordinates of new point as Point2

<a id="mapping_common.mask.create_lane_box"></a>

### mapping_common.mask.create_lane_box(y_axis_line, lane_close_hero, lane_further_hero, lane_pos, lane_length, lane_transform, reduce_lane)

helper function to create a lane box entity

* **Return type:**
  `Geometry`

Args:
: y_axis_line (LineString): check shape y-axis line
  lane_close_hero (Entity): the lane marking entity that is closer to the car
  lane_further_hero (Entity): the lane marking entity that is further away
  <br/>
  > from the car
  <br/>
  lane_pos (int): to check if the lane is on the left or right side of the car
  lane_length (float): length of the lane box
  lane_transform (float): transform of the lane box
  reduce_lane (float): reduce the lane

Returns:
: lane_box (Geometry): created lane box shape
