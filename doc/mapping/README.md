# Mapping package / Intermediate layer

**Summary:** The intermediate layer is a 2D top-down map for sensor fusion and entity tracking.
The map can be visualized with the visualization node in the [mapping_visualization](/code/mapping_visualization/) package

- [Package structure](#package-structure)
- [Data flow overview](#data-flow-overview)
- [Tests](#tests)
- [Research](#research)
- [mapping\_common cython installation](#mapping_common-cython-installation)
- [Debugging](#debugging)
- [Auto-generated documentation](#auto-generated-documentation)
- [Troubleshooting](#troubleshooting)

## Package structure

- [./src](/code/mapping/src/) contains the nodes that create and filter the map
  - The main node is the [**MappingDataIntegrationNode**](/doc/mapping/generated/nodes.md#mappingdataintegrationnode). It collects sensor data and then build a map from it
  - **[API documentation](/doc/mapping/generated/nodes.md)**
- [./msg](/code/mapping/msg/) contains the ROS message types for transmitting the map
- [./ext_modules/mapping_common](/code/mapping/ext_modules/mapping_common/) contains the **python classes for working with the map**.
  - ROS messages can be converted into these python classes by using `<Type>.from_ros_msg(msg)`
  - **[API documentation](/doc/mapping/generated/mapping_common/index.md)**
  - This module is compiled with Cython. If changes have been made to this package, catkin_make needs to be executed to apply them! \
    This step is automatically executed when using the [docker-compose.leaderboard.yaml](/build/docker-compose.leaderboard.yaml).

The base data type is the [Map](/doc/mapping/generated/mapping_common/map.md#map). It consists out of [Entities](/doc/mapping/generated/mapping_common/entity.md#entity).

These entities all have a [transform](/doc/mapping/generated/mapping_common/transform.md#transform2d) and a [shape](/doc/mapping/generated/mapping_common/shape.md#shape2d) and can be all kinds of colliders (car, pedestrian, etc.), lanemarkings or other localized things of interest around the hero car.

## Data flow overview

Quick overview of the data flow inside the intermediate layer

```mermaid
flowchart TD
    NF{New empty dataframe} --> A[node: mapping_data_integration]
    S{Sensors} --> A
    A --> F1(filter: <a href='/doc/mapping/generated/mapping_common/filter.md#growthmergingfilter'>GrowthMergingFilter</a>)
    F1 --> F2(filter: <a href='/doc/mapping/generated/mapping_common/filter.md#laneindexfilter'>LaneIndexFilter</a>)
    F2 --> F3(filter: <a href='/doc/mapping/generated/mapping_common/filter.md#growpedestriansfilter'>GrowPedestriansFilter</a>)
    F3 -->|topic: /paf/hero/mapping/init_data| 1[node: <a href='/doc/mapping/generated/nodes.md#mappingdataintegrationnode'>mapping_visualization</a>]
```

This information is kept up-to-date with the package

## Tests

This package contains pytest based unit tests at [./tests/mapping_common](/code/mapping/tests/mapping_common/)

The tests can be executed without a running ros/carla instance.

Execute `catkin_make run_tests` in the catkin_ws to run them. A summary should appear there.

## Research

The original draft and class diagram for the intermediate layer can be found [here](/doc/research/paf24/intermediate_layer/). Note that these documents are not kept up-to-date with this package.

Most of the information of the draft has been inserted into the python class documentation. Look there for up-to-date information.

## mapping_common cython installation

**Important: The mapping_common module is compiled with [Cython](https://cython.readthedocs.io/en/latest/). If changes have been made to mapping_common, catkin_make needs to be executed to apply them!**
This step is automatically executed when using the [docker-compose.leaderboard.yaml](/build/docker-compose.leaderboard.yaml).

Cmake executes [cmake_setup.py](/code/mapping/ext_modules/cmake_setup.py) to compile and install the mapping_common module.

The integration of the command into catkin_make can be found [here](/code/mapping/CMakeLists.txt#L100).

## Debugging

The debugger is unable to debug the mapping_common package when it is compiled.

To disable the compilation, you can replace the `False` in [./ext_modules/.debug_enabled](/code/mapping/ext_modules/.debug_enabled) with `True`.

## Auto-generated documentation

The markdown files in the [./generated](/doc/mapping/generated/) folder are generated with `pydoc-markdown`.

To regenerate them, run the [docker-compose.docs.yaml](/build/docker-compose.docs.yaml).

## Troubleshooting

Cython compile errors or import errors *related to the @dataclass decorators* are likely caused by an outdated docker image. Make sure the docker images are up-to-date.
