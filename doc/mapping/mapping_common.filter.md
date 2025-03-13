# mapping_common.filter module

<a id="mapping_common.filter.MapFilter"></a>

### *class* mapping_common.filter.MapFilter

Bases: `object`

Abstract base class for all mapping filters

<a id="mapping_common.filter.MapFilter.filter"></a>

#### filter(map)

Filters the map.

Look into the class description for what the filter does

* **Return type:**
  [`Map`](mapping_common.map.md#mapping_common.map.Map)

Args:
: map (Map): Map to filter

Returns:
: Map: New map with filter applied.
  : Note that unmodified entities are NOT deepcopied.

<a id="mapping_common.filter.LaneIndexFilter"></a>

### *class* mapping_common.filter.LaneIndexFilter

Bases: [`MapFilter`](#mapping_common.filter.MapFilter)

Updates the Index of lanemark Entities if duplicates have been removed.

!!!Must be called after GrowthMergingFilter!!!

- Calculates the y coordinates of the intersection with y axis of each lanemarking.
- Gives position_index according to y position:
  : - 1 = lane next to the car on the left.
    - 2 = second lanemark on the left.
    - -1 = lane next to the car on the right.
    - etc.

Then returns the updated map with all Entities

<a id="mapping_common.filter.LaneIndexFilter.filter"></a>

#### filter(map)

Filters the map.

Look into the class description for what the filter does

* **Return type:**
  [`Map`](mapping_common.map.md#mapping_common.map.Map)

Args:
: map (Map): Map to filter

Returns:
: Map: New map with filter applied.
  : Note that unmodified entities are NOT deepcopied.

<a id="mapping_common.filter.LaneIndexFilter.__init__"></a>

#### \_\_init_\_()

<a id="mapping_common.filter.GrowPedestriansFilter"></a>

### *class* mapping_common.filter.GrowPedestriansFilter

Bases: [`MapFilter`](#mapping_common.filter.MapFilter)

Grow Pedestrians by 0.5 meter for a better detection of them (e.g. for the ACC)

!!!Must be called after GrowthMergingFilter!!!

- Iterates over all entities
- If entity is a Pedestrian: Grow them by 0.5 meter

Then returns the updated map with all Entities

<a id="mapping_common.filter.GrowPedestriansFilter.filter"></a>

#### filter(map)

Filters the map.

Look into the class description for what the filter does

* **Return type:**
  [`Map`](mapping_common.map.md#mapping_common.map.Map)

Args:
: map (Map): Map to filter

Returns:
: Map: New map with filter applied.
  : Note that unmodified entities are NOT deepcopied.

<a id="mapping_common.filter.GrowPedestriansFilter.__init__"></a>

#### \_\_init_\_()

<a id="mapping_common.filter.GrowthMergingFilter"></a>

### *class* mapping_common.filter.GrowthMergingFilter(growth_distance, min_merging_overlap_percent, min_merging_overlap_area, simplify_tolerance)

Bases: [`MapFilter`](#mapping_common.filter.MapFilter)

Merges entities in the map with growing them

Basic (very simplified) function:
- Compares pairs of entities that are in the vicinity of each other
- For each pair:

> - Checks if they are mergeable at all
> - Grows their shape based on growth_distance
>   : -> This is done in order to also catch small noisy entities
>     (outside) around a bigger entity
> - Checks the min_merging_overlap_percent and min_merging_overlap_area
>   : based on the intersection of the grown shapes
> - -> Merge if at least one of them is true
> - Creates a grown union for the merged entity
> - Shrinks the union by growth_distance
- It then deletes all entities that got merged into another
  : and returns the resulting map

<a id="mapping_common.filter.GrowthMergingFilter.growth_distance"></a>

#### growth_distance *: `float`*

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_percent"></a>

#### min_merging_overlap_percent *: `float`*

Min overlap of the grown shapes in percent

<a id="mapping_common.filter.GrowthMergingFilter.min_merging_overlap_area"></a>

#### min_merging_overlap_area *: `float`*

Min overlap of the grown shapes in m2

<a id="mapping_common.filter.GrowthMergingFilter.simplify_tolerance"></a>

#### simplify_tolerance *: `float`*

<a id="mapping_common.filter.GrowthMergingFilter.filter"></a>

#### filter(map)

Filters the map.

Look into the class description for what the filter does

* **Return type:**
  [`Map`](mapping_common.map.md#mapping_common.map.Map)

Args:
: map (Map): Map to filter

Returns:
: Map: New map with filter applied.
  : Note that unmodified entities are NOT deepcopied.

<a id="mapping_common.filter.GrowthMergingFilter.__init__"></a>

#### \_\_init_\_(growth_distance, min_merging_overlap_percent, min_merging_overlap_area, simplify_tolerance)
