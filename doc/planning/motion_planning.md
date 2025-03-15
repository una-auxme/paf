# Motion Planning

**Summary:** [motion_planning.py](../../code/planning/src/local_planner/motion_planning.py):
The motion planning is responsible for collecting all the velocity recommendations from the different components and choosing the optimal one to be fowarded into the acting. It also is capable to change the trajectory for an overtaking maneuver.

- [Overview](#overview)
- [ROS Data Interface](#ros-data-interface)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
- [Node Creation + Running Tests](#node-creation--running-tests)

## Overview

This module contains one ROS node and is responsible for adjusting and publishing the target velocity and the target trajectory according to the traffic situation.
It subscribes to topics that provide information about the current velocity of the vehicle, the current heading and many more to navigate safely in the simulation.
It also publishes a topic that indicates wether an overtake maneuver was successful or not.

This component is also responsible for providing a target_speed of `-3` for acting, whenever we need to use the Unstuck Behavior. This is currently the only behavior that allows the car to drive backwards.

The trajectory is calculated by the global planner and is adjusted by the motion planning node.
When the decision making node decides that an overtake maneuver is necessary, the motion planning node will adjust the trajectory accordingly.
Otherwise the received trajectory is published without any changes.

## ROS Data Interface

### Subscribed Topics

This node subscribes to the following topics:

- `/carla/hero/Speed`: Subscribes to the current speed.
- `/paf/hero/acc_velocity`: Subscribes to the speed published by the acc.
- `/paf/hero/Center/traffic_light_y_distance`: Subscribes to the distance the traffic light has to the upper camera bound in pixels.
- `/paf/hero/curr_behavior`: Subscribes to Current Behavior pubished by the Decision Making.
- `/paf/hero/current_heading`: Subscribes to the filtered heading of the ego vehicle.
- `/paf/hero/current_pos`: Subscribes to the filtered position of our car.
- `/paf/hero/current_wp`: Subscribes to the current waypoint.
- `/paf/hero/lane_change`: Subscribes to the Carla Waypoint to check if the next Waypoint is a lane change.
- `/paf/hero/speed_limit`: Subscribes to the speed limit.
- `/paf/hero/trajectory_global`: Subscribes to the global trajectory, which is calculated at the start of the simulation.
- `/paf/hero/unstuck_distance`: Subscribes to the distance travelled by the unstuck maneuver.
- `/paf/hero/current_waypoint`: Subscribes to the Carla Waypoint to get the new road option.

### Published Topics

This node publishes the following topics:

- `/paf/hero/overtake_success`: Publishes if an overtake was successful. ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- `/paf/hero/target_velocity`: Publishes the target velocity in m/s. ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- `/paf/hero/trajectory`: Publishes the new adjusted trajectory. ([nav_msgs/Path](https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))

## Node Creation + Running Tests

To run this node insert the following statement in the [planning.launch](../../code/planning/launch/planning.launch) file:

```xml
<node pkg="planning" type="motion_planning.py" name="MotionPlanning" output="screen">
           <param name="role_name" value="hero" />
           <param name="control_loop_rate" value="0.1" />
</node>
```
