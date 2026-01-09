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

