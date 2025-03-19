<!-- markdownlint-disable -->
# Filter documentation

## Table of Contents

* [mapping\_common.filter](#mapping_common.filter)
  * [MapFilter](#mapping_common.filter.MapFilter)
    * [filter](#mapping_common.filter.MapFilter.filter)
  * [LaneIndexFilter](#mapping_common.filter.LaneIndexFilter)
  * [GrowPedestriansFilter](#mapping_common.filter.GrowPedestriansFilter)
  * [GrowthMergingFilter](#mapping_common.filter.GrowthMergingFilter)
    * [min\_merging\_overlap\_percent](#mapping_common.filter.GrowthMergingFilter.min_merging_overlap_percent)
    * [min\_merging\_overlap\_area](#mapping_common.filter.GrowthMergingFilter.min_merging_overlap_area)

<a id="mapping_common.filter"></a>

# mapping\_common.filter

Contains filter-related functions

**[API documentation](/doc/mapping/generated/mapping_common/filter.md)**

<a id="mapping_common.filter.MapFilter"></a>

## MapFilter

```python
class MapFilter()
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/filter.py#L20)

Abstract base class for all mapping filters

<a id="mapping_common.filter.MapFilter.filter"></a>

#### filter

```python
def filter(map: Map) -> Map
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/filter.py#L23)

Filters the map.

Look into the class description for what the filter does

**Arguments**:

- `map` _Map_ - Map to filter
  

**Returns**:

- `Map` - New map with filter applied.
  Note that unmodified entities are NOT deepcopied.

<a id="mapping_common.filter.LaneIndexFilter"></a>

## LaneIndexFilter

```python
@dataclass
class LaneIndexFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/filter.py#L39)

Updates the Index of lanemark Entities if duplicates have been removed.

!!!Must be called after GrowthMergingFilter!!!

- Calculates the y coordinates of the intersection with y axis of each lanemarking.
- Gives position_index according to y position:
    - 1 = lane next to the car on the left.
    - 2 = second lanemark on the left.
    - -1 = lane next to the car on the right.
    - etc.

Then returns the updated map with all Entities

<a id="mapping_common.filter.GrowPedestriansFilter"></a>

## GrowPedestriansFilter

```python
@dataclass
class GrowPedestriansFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/filter.py#L90)

Grow Pedestrians by 0.5 meter for a better detection of them (e.g. for the ACC)

!!!Must be called after GrowthMergingFilter!!!

- Iterates over all entities
- If entity is a Pedestrian: Grow them by 0.5 meter

Then returns the updated map with all Entities

<a id="mapping_common.filter.GrowthMergingFilter"></a>

## GrowthMergingFilter

```python
@dataclass
class GrowthMergingFilter(MapFilter)
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/filter.py#L111)

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

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_percent"></a>

#### min\_merging\_overlap\_percent: `float`

Min overlap of the grown shapes in percent

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_area"></a>

#### min\_merging\_overlap\_area: `float`

Min overlap of the grown shapes in m2

