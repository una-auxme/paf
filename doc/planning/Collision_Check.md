# Collision Check

**Summary:** This module is responsible for detecting collisions and reporting them. It subscribes to topics that provide information about the current speed of the vehicle and the distances to objects detected by a LIDAR sensor.
It publishes topics that provide information about emergency stops, the distance to collisions, the distance to oncoming traffic, and the approximated speed of the obstacle in front.

- [Component](#component)
- [ROS Data Interface](#ros-data-interface)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
- [Node Creation + Running Tests](#node-creation--running-tests)

## Component

The Collision Check only consists of one node that contains all subscriper and publishers. It uses some utility functions from [utils.py](../../code//planning/src/local_planner/utils.py).

## ROS Data Interface

### Published Topics

- `/paf/hero/emergency`: Published when an emergency stop is required.
- `/paf/hero/collision`: Publishes the distance and speed from a collision object.
- `/paf/hero/oncoming`: Publishes the distance to oncoming traffic.

### Subscribed Topics

- `/carla/hero/Speed`: Subscribes to the current speed of the vehicle.
- `/paf/hero/Center/object_distance`: Subscribes to the distances to objects detected by a LIDAR sensor.

## Node Creation + Running Tests

To run this node insert the following statement in the [planning.launch](../../code/planning/launch/planning.launch) file:

```xml
<node pkg="planning" type="collision_check.py" name="CollisionCheck" output="screen">
        <param name="role_name" value="hero" />
        <param name="control_loop_rate" value="0.05" />
</node>
```
