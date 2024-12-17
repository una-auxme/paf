# Intermediate layer

The intermediate layer is a 2D top-down map for sensor fusion and object tracking.

## Package structure

- [./src](./src/) contains the nodes that create and filter the map
- [./msg](./msg/) contains the ROS message types for transmitting the map
- [./ext_modules/mapping_common](./ext_modules/mapping_common/) contains the **python classes for working with the map**.
  ROS messages can be converted into these python classes by using `<Type>.from_ros_msg(msg)`

The base data type is the [Map](./ext_modules/mapping_common/map.py). It consists out of [Entities](./ext_modules/mapping_common/entity.py).

These entities all have a transform and a shape and can be all kinds of collisions (car, pedestrian, etc.), lanemarkings or other localized things of interest around the hero car.

## Research

The original draft and class diagram for the intermediate layer can be found [here](../../doc/research/paf24/intermediate_layer/). Note that these documents are not kept up-to-date with this package. Look into the python class documentation for up-to-date information.
