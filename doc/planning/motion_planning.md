# Motion Planning

**Summary:** [motion_planning.py](.../code/planning/local_planner/src/motion_planning.py):
The motion planning is responsible for collecting all the speeds from the different components and choosing the optimal one to be fowarded into the acting. It also is capable to change the trajectory for an overtaking maneuver.

- [Overview](#overview)
- [Component](#component)
- [ROS Data Interface](#ros-data-interface)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
- [Node Creation + Running Tests](#node-creation--running-tests)

## Overview

This module is responsible for adjusting the current speed and the current trajectory according to the traffic situation. It subscribes to topics that provide information about the current speed of the vehicle, the current heading and many more to navigate safely in the simulation.
It publishes topics that provide information about the target speed, trajectoy changes, current waypoint and if an overtake was successful.

This file is also responsible for providing a ```target_speed of -3``` for acting, whenever we need to use the Unstuck Behavior. -3 is the only case we can drive backwards right now,
since we only need it for the unstuck routine. It also creates an overtake trajectory, whenever the unstuck behavior calls for it.

## Component

The Motion Planning only consists of one node that contains all subscribers and publishers. It uses some utility functions from [utils.py](../../code/planning/src/local_planner/utils.py).

## ROS Data Interface

### Subscribed Topics

This node subscribes to the following topics:

- `/paf/hero/speed_limit`: Subscribes to the speed Limit.
- `/carla/hero/Speed`: Subscribes to the current speed.
- `/paf/hero/current_heading`: Subscribes to the filtered heading of the ego vehicle.
- `/paf/hero/trajectory_global`: Subscribes to the global trajectory, which is calculated at the start of the simulation.
- `/paf/hero/current_pos`: Subscribes to the filtered position of our car.
- `/paf/hero/curr_behavior`: Subscribes to Current Behavior pubished by the Decision Making.
- `/paf/hero/acc_velocity`: Subscribes to the speed published by the acc.
- `/paf/hero/waypoint_distance`: Subscribes to the Carla Waypoint to get the new road option.
- `/paf/hero/lane_change_distance`: Subscribes to the Carla Waypoint to check if the next Waypoint is a lane change.
- `/paf/hero/collision`: Subscribes to the collision published by the Collision Check.
- `/paf/hero/Center/traffic_light_y_distance`: Subscribes to the distance the traffic light has to the upper camera bound in pixels.
- `/paf/hero/unstuck_distance`: Subscribes to the distance travelled by the unstuck maneuver.

### Published Topics

This node publishes the following topics:

- `/paf/hero/trajectory`: Publishes the new adjusted trajectory.
- `/paf/hero/target_velocity`: Publishes the new calcualted Speed.
- `/paf/hero/current_wp`: Publishes according to our position the index of the current point on the trajectory.
- `/paf/hero/overtake_success`: Publishes if an overtake was successful.

## Node Creation + Running Tests

To run this node insert the following statement in the [planning.launch](../../code/planning/launch/planning.launch) file:

```xml
<node pkg="planning" type="motion_planning.py" name="MotionPlanning" output="screen">
           <param name="role_name" value="hero" />
           <param name="control_loop_rate" value="0.1" />
</node>
```

The motion planning node listens to the following debugging topics:

- `/paf/hero/Spawn_car`: Can spawn a car on the first straight in the dev environment, if this message is manually published.
