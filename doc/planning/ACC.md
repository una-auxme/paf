# ACC (Adaptive Cruise Control)

**Summary:** The ACC module is a ROS node responsible for adaptive speed control in an autonomous vehicle. It receives information about possible collisions, the current speed, the trajectory, and the speed limits. Based on this information, it calculates the desired speed and publishes it.

- [ROS Data Interface](#ros-data-interface)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
- [Node Creation + Running Tests](#node-creation--running-tests)

## ROS Data Interface

### Published Topics

This module publishes the following topics:

- `/paf/hero/acc_velocity`: The desired speed for the vehicle.
- `/paf/hero/current_wp`: The current waypoint.
- `/paf/hero/speed_limit`: The current speed limit.

### Subscribed Topics

This module subscribes to the following topics:

- `/paf/hero/unstuck_flag`: A flag indicating whether the vehicle is stuck.
- `/paf/hero/unstuck_distance`: The distance the vehicle needs to travel to get unstuck.
- `/carla/hero/Speed`: The current speed of the vehicle.
- `/paf/hero/speed_limits_OpenDrive`: The speed limits from OpenDrive.
- `/paf/hero/trajectory_global`: The global trajectory of the vehicle.
- `/paf/hero/current_pos`: The current position of the vehicle.
- `/paf/hero/collision`: Information about a possible collision object.
- `/paf/hero/Radar/lead_vehicle/range_velocity_array`: Distance to the vehicle in front, relative speed of the vehicle in front

## Node Creation + Running Tests

To run this node insert the following statement in the [planning.launch](../../code/planning/launch/planning.launch) file:

```xml
<node pkg="planning" type="ACC.py" name="ACC" output="screen">
    <param name="role_name" value="hero" />
    <param name="control_loop_rate" value="0.3" />
</node>
```
