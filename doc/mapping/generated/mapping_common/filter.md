<!-- markdownlint-disable -->
# Filter documentation

## Table of Contents

* [mapping\_common.filter](#mapping_common.filter)
  * [MapFilter](#mapping_common.filter.MapFilter)
    * [filter](#mapping_common.filter.MapFilter.filter)
  * [LaneIndexFilter](#mapping_common.filter.LaneIndexFilter)
    * [filter](#mapping_common.filter.LaneIndexFilter.filter)
  * [GrowPedestriansFilter](#mapping_common.filter.GrowPedestriansFilter)
    * [filter](#mapping_common.filter.GrowPedestriansFilter.filter)
  * [GrowthMergingFilter](#mapping_common.filter.GrowthMergingFilter)
    * [growth\_distance](#mapping_common.filter.GrowthMergingFilter.growth_distance)
    * [min\_merging\_overlap\_percent](#mapping_common.filter.GrowthMergingFilter.min_merging_overlap_percent)
    * [min\_merging\_overlap\_area](#mapping_common.filter.GrowthMergingFilter.min_merging_overlap_area)
    * [simplify\_tolerance](#mapping_common.filter.GrowthMergingFilter.simplify_tolerance)
    * [filter](#mapping_common.filter.GrowthMergingFilter.filter)
  * [TrackingFilter](#mapping_common.filter.TrackingFilter)
    * [max\_distance\_threshold](#mapping_common.filter.TrackingFilter.max_distance_threshold)
    * [max\_distance](#mapping_common.filter.TrackingFilter.max_distance)
    * [cur\_entities](#mapping_common.filter.TrackingFilter.cur_entities)
    * [prev1\_entities](#mapping_common.filter.TrackingFilter.prev1_entities)
    * [prev2\_entities](#mapping_common.filter.TrackingFilter.prev2_entities)
    * [update\_tracking\_velocity](#mapping_common.filter.TrackingFilter.update_tracking_velocity)
    * [ego\_delta\_heading](#mapping_common.filter.TrackingFilter.ego_delta_heading)
    * [\_get\_entity\_position](#mapping_common.filter.TrackingFilter._get_entity_position)
    * [\_euclidean\_distance](#mapping_common.filter.TrackingFilter._euclidean_distance)
    * [set\_tracking\_velocity\_status](#mapping_common.filter.TrackingFilter.set_tracking_velocity_status)
    * [set\_delta\_heading](#mapping_common.filter.TrackingFilter.set_delta_heading)
    * [\_assign\_new\_track\_id](#mapping_common.filter.TrackingFilter._assign_new_track_id)
    * [update\_tracked\_entity](#mapping_common.filter.TrackingFilter.update_tracked_entity)
    * [\_track\_entities](#mapping_common.filter.TrackingFilter._track_entities)
    * [check\_valid\_entities\_data](#mapping_common.filter.TrackingFilter.check_valid_entities_data)
    * [set\_entities\_data](#mapping_common.filter.TrackingFilter.set_entities_data)
    * [filter](#mapping_common.filter.TrackingFilter.filter)
  * [RadarPointAssignmentFilter](#mapping_common.filter.RadarPointAssignmentFilter)
    * [radar\_compensated\_points\_data](#mapping_common.filter.RadarPointAssignmentFilter.radar_compensated_points_data)
    * [classification\_threshold](#mapping_common.filter.RadarPointAssignmentFilter.classification_threshold)
    * [radar\_lidar\_assoc\_buffer](#mapping_common.filter.RadarPointAssignmentFilter.radar_lidar_assoc_buffer)
    * [set\_radar\_lidar\_assoc\_buffer](#mapping_common.filter.RadarPointAssignmentFilter.set_radar_lidar_assoc_buffer)
    * [set\_classification\_threshold](#mapping_common.filter.RadarPointAssignmentFilter.set_classification_threshold)
    * [set\_radar\_points](#mapping_common.filter.RadarPointAssignmentFilter.set_radar_points)
    * [fuse\_radar\_velocity\_into\_lidar\_entities](#mapping_common.filter.RadarPointAssignmentFilter.fuse_radar_velocity_into_lidar_entities)
    * [check\_valid\_data](#mapping_common.filter.RadarPointAssignmentFilter.check_valid_data)
    * [filter](#mapping_common.filter.RadarPointAssignmentFilter.filter)

<a id="mapping_common.filter"></a>

# mapping\_common.filter

Contains the postprocessing filters for the **Intermediate Layer**

**[API documentation](/doc/mapping/generated/mapping_common/filter.md)**

After collecting the sensor data in the *mapping_data_integration* node,
several filters are applied to improve the Map before
sending it to Planning/Acting.

This module contains these filters and algorithms.

<a id="mapping_common.filter.MapFilter"></a>

## MapFilter

```python
class MapFilter()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L26)

Abstract base class for all mapping filters

<a id="mapping_common.filter.MapFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L29)

Filters the map.

Look into the class description for what the filter does

**Arguments**:

- `map` _Map_ - Map to filter
  

**Returns**:

- `Map` - New map with filter applied.
  Note that unmodified entities might NOT be deepcopied.

<a id="mapping_common.filter.LaneIndexFilter"></a>

## LaneIndexFilter

```python
@dataclass
class LaneIndexFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L45)

Updates the Index of lanemark Entities if duplicates have been removed.

!!!Must be called after GrowthMergingFilter!!!

- Calculates the y coordinates of the intersection with y axis of each lanemarking.
- Gives position_index according to y position:
    - 1 = lane next to the car on the left.
    - 2 = second lanemark on the left.
    - -1 = lane next to the car on the right.
    - etc.

Then returns the updated map with all Entities

<a id="mapping_common.filter.LaneIndexFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L60)

<a id="mapping_common.filter.GrowPedestriansFilter"></a>

## GrowPedestriansFilter

```python
@dataclass
class GrowPedestriansFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L96)

Grow Pedestrians by 0.5 meter for a better detection of them (e.g. for the ACC)

!!!Must be called after GrowthMergingFilter!!!

- Iterates over all entities
- If entity is a Pedestrian: Grow them by 0.5 meter

Then returns the updated map with all Entities

<a id="mapping_common.filter.GrowPedestriansFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L107)

<a id="mapping_common.filter.GrowthMergingFilter"></a>

## GrowthMergingFilter

```python
@dataclass
class GrowthMergingFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L117)

Merges entities in the map with growing them

Basic (very simplified) function:
- Compares pairs of entities that are in the vicinity of each other
- For each pair:
    - Checks if they are mergeable at all
    - Grows their shape based on growth_distance
        -> This is done in order to also catch small noisy entities
        (outside) around a bigger entity
    - Checks the min_merging_overlap_percent and min_merging_overlap_area
        based on the intersection of the grown shapes
    - -> Merge if at least one of them is true
    - Creates a grown union for the merged entity
    - Shrinks the union by growth_distance
- It then deletes all entities that got merged into another
    and returns the resulting map

<a id="mapping_common.filter.GrowthMergingFilter.growth_distance"></a>

#### growth\_distance: `float`

Growth amount in m

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_percent"></a>

#### min\_merging\_overlap\_percent: `float`

Min overlap of the grown shapes in percent

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_area"></a>

#### min\_merging\_overlap\_area: `float`

Min overlap of the grown shapes in m2

<a id="mapping_common.filter.GrowthMergingFilter.simplify_tolerance"></a>

#### simplify\_tolerance: `float`

The shapes are simplified after each merge.

This controls the tolerance of the simplification.

<a id="mapping_common.filter.GrowthMergingFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L152)

<a id="mapping_common.filter.TrackingFilter"></a>

## TrackingFilter

```python
class TrackingFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L237)

Implements multi-frame tracking for Map Entities using the Hungarian algorithm
and two-frame history (prev1 and prev2) for robustness against noisy data or
brief occlusions.

Key Features:
1. Two-Stage Matching: Attempts to match current entities against the immediately
   previous frame (prev1) first, and then against the frame before that (prev2)
   for unmatched entities.
2. Type Persistence: Ensures that a classified entity (e.g., Car) retains its
   type and specific attributes even if it momentarily becomes a generic Entity
   (e.g., when moving out of the camera's field of view).
3. Lanemarking Exclusion: Excludes static Lanemarking entities from the tracking
   overhead and safely recombines them in the final map.

<a id="mapping_common.filter.TrackingFilter.max_distance_threshold"></a>

#### max\_distance\_threshold: `float`

Maximum Euclidean distance (m) for two entities to be considered a match.

<a id="mapping_common.filter.TrackingFilter.max_distance"></a>

#### max\_distance: `float`

<a id="mapping_common.filter.TrackingFilter.cur_entities"></a>

#### cur\_entities: `Optional[List[Entity]]`

Entities from the current frame (will be matched and modified).

<a id="mapping_common.filter.TrackingFilter.prev1_entities"></a>

#### prev1\_entities: `Optional[List[Entity]]`

Entities from the previous frame (t-1).

<a id="mapping_common.filter.TrackingFilter.prev2_entities"></a>

#### prev2\_entities: `Optional[List[Entity]]`

Entities from the frame before the previous one (t-2).

<a id="mapping_common.filter.TrackingFilter.update_tracking_velocity"></a>

#### update\_tracking\_velocity: `bool`

Global toggle for enabling/disabling velocity computation.

<a id="mapping_common.filter.TrackingFilter.ego_delta_heading"></a>

#### ego\_delta\_heading: `float`

<a id="mapping_common.filter.TrackingFilter._get_entity_position"></a>

#### \_get\_entity\_position

```python
@staticmethod
def _get_entity_position(entity: Entity) -> Vector2
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#275)

<a id="mapping_common.filter.TrackingFilter._euclidean_distance"></a>

#### \_euclidean\_distance

```python
@staticmethod
def _euclidean_distance(pos1: Vector2, pos2: Vector2) -> float
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L280)

<a id="mapping_common.filter.TrackingFilter.set_tracking_velocity_status"></a>

#### set\_tracking\_velocity\_status

```python
def set_tracking_velocity_status(track_velocity: bool)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L290)

Updates the configuration to enable/disable velocity calculation.

<a id="mapping_common.filter.TrackingFilter.set_delta_heading"></a>

#### set\_delta\_heading

```python
def set_delta_heading(delta_heading: float)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#294)

<a id="mapping_common.filter.TrackingFilter._assign_new_track_id"></a>

#### \_assign\_new\_track\_id

```python
def _assign_new_track_id(entity: Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L297)

Initializes tracking info for a new, unmatched entity.

<a id="mapping_common.filter.TrackingFilter.update_tracked_entity"></a>

#### update\_tracked\_entity

```python
def update_tracked_entity(cur_entity: Entity, prev_entity: Entity)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L327)

Updates the current entity with persistent tracking information from the
previous entity, including UUID, history, and entity type.

<a id="mapping_common.filter.TrackingFilter._track_entities"></a>

#### \_track\_entities

```python
def _track_entities(prev_entities: List[Entity],
                    cur_entities: List[Entity]) -> Set[int]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L389)

Matches current entities against a list of previous candidates
using the Hungarian algorithm.

**Arguments**:

- `prev_entities` _List[Entity]_ - Entities from a previous frame (prev1 or prev2).
- `cur_entities` _List[Entity]_ - Current entities to match (modified in-place).

**Returns**:

- `Set[int]` - Indices of cur_entities that were successfully matched.

<a id="mapping_common.filter.TrackingFilter.check_valid_entities_data"></a>

#### check\_valid\_entities\_data

```python
def check_valid_entities_data()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L437)

Checks if there is enough history (at least one previous frame)
to start tracking.

<a id="mapping_common.filter.TrackingFilter.set_entities_data"></a>

#### set\_entities\_data

```python
def set_entities_data(entities: List[Entity])
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L453)

Shifts the map history: cur -> prev1, prev1 -> prev2

<a id="mapping_common.filter.TrackingFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L459)

Filters the map by performing two-stage tracking
and re-assigning persistent IDs.

<a id="mapping_common.filter.RadarPointAssignmentFilter"></a>

## RadarPointAssignmentFilter

```python
class RadarPointAssignmentFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L538)

Assigns radar-derived velocities to LiDAR entities using spatial association.

<a id="mapping_common.filter.RadarPointAssignmentFilter.radar_compensated_points_data"></a>

#### radar\_compensated\_points\_data

<a id="mapping_common.filter.RadarPointAssignmentFilter.classification_threshold"></a>

#### classification\_threshold

<a id="mapping_common.filter.RadarPointAssignmentFilter.radar_lidar_assoc_buffer"></a>

#### radar\_lidar\_assoc\_buffer

<a id="mapping_common.filter.RadarPointAssignmentFilter.set_radar_lidar_assoc_buffer"></a>

#### set\_radar\_lidar\_assoc\_buffer

```python
def set_radar_lidar_assoc_buffer(radar_lidar_assoc_buffer)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L547)

<a id="mapping_common.filter.RadarPointAssignmentFilter.set_classification_threshold"></a>

#### set\_classification\_threshold

```python
def set_classification_threshold(classification_threshold)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L550)

<a id="mapping_common.filter.RadarPointAssignmentFilter.set_radar_points"></a>

#### set\_radar\_points

```python
def set_radar_points(radar_compensated_points_data)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L553)

<a id="mapping_common.filter.RadarPointAssignmentFilter.fuse_radar_velocity_into_lidar_entities"></a>

#### fuse\_radar\_velocity\_into\_lidar\_entities

```python
def fuse_radar_velocity_into_lidar_entities(lidar_entities: List[Entity]) -> None
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L556)

<a id="mapping_common.filter.RadarPointAssignmentFilter.check_valid_data"></a>

#### check\_valid\_data

```python
def check_valid_data()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L620)

<a id="mapping_common.filter.RadarPointAssignmentFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/filter.py#L624)

