<!-- markdownlint-disable -->
# Mapping Common

## Table of Contents

* [mapping\_common](#mapping_common)
* [mapping\_common.markers](#mapping_common.markers)
* [mapping\_common.hero](#mapping_common.hero)
* [mapping\_common.map](#mapping_common.map)
* [mapping\_common.shape](#mapping_common.shape)
* [mapping\_common.transform](#mapping_common.transform)
* [mapping\_common.entity](#mapping_common.entity)
* [mapping\_common.filter](#mapping_common.filter)
* [mapping\_common.mask](#mapping_common.mask)

<a id="mapping_common"></a>

# mapping\_common



<a id="mapping_common.markers"></a>

# mapping\_common.markers

Contains functions to easily create debug markers that can be visualized in RViz

**[API documentation](/doc/mapping/generated/mapping_common/markers.md)**

Overview of the main components:
- debug_marker(): Creates a ROS Marker based on different objects
- debug_marker_array(): Creates a ROS MarkerArray
  based on list of ROS Markers

<a id="mapping_common.hero"></a>

# mapping\_common.hero

Contains attributes and functions specific to the hero

**[API documentation](/doc/mapping/generated/mapping_common/hero.md)**

Currently available:
- Shape: Length/Width of the Hero based on the Lincoln MKZ 2020
- Function to create a basic hero Entity

<a id="mapping_common.map"></a>

# mapping\_common.map

Contains the Map class and functions for working with it

**[API documentation](/doc/mapping/generated/mapping_common/map.md)**

Overview of the main components:
- **Map** class: Main datatype transmitted in the Intermediate Layer
- MapTree: Acceleration structure for intersection checks on the Map.
  Also includes all lane check functions used in the
  behavior tree.

<a id="mapping_common.shape"></a>

# mapping\_common.shape

Contains shape-related functions

**[API documentation](/doc/mapping/generated/mapping_common/shape.md)**

<a id="mapping_common.transform"></a>

# mapping\_common.transform

Contains transform-related functions

**[API documentation](/doc/mapping/generated/mapping_common/transform.md)**

<a id="mapping_common.entity"></a>

# mapping\_common.entity

Contains Entity classes and functions

**[API documentation](/doc/mapping/generated/mapping_common/entity.md)**

Overview of the main components:
- **Entity** class and subclasses (Car, Pedestrian, etc..)
- Entity attribute classes: Motion2D, Flags, TrackingInfo
- ShapelyEntity: Container containing an Entity and its shape as shapely.Polygon

<a id="mapping_common.filter"></a>

# mapping\_common.filter

Contains the postprocessing filters for the **Intermediate Layer**

**[API documentation](/doc/mapping/generated/mapping_common/filter.md)**

After collecting the sensor data in the *mapping_data_integration* node,
several filters are applied to improve the Map before
sending it to Planning/Acting.

This module contains these filters and algorithms.

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

