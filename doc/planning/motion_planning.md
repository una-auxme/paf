# Motion Planning

**Summary:** [motion_planning.py](../../code/planning/src/local_planner/motion_planning.py):
The motion planning is responsible for collecting all the velocity recommendations from the different components and choosing the optimal one to be fowarded into the acting. It also is capable to change the trajectory for an overtaking maneuver.

- [Overview](#overview)
- [ROS Data Interface](#ros-data-interface)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
  - [Provided Services](#provided-services)
- [Node Creation + Running Tests](#node-creation--running-tests)

## Overview

This module contains one ROS node and is responsible for adjusting and publishing the speed limit, the trajectory and the local trajectory according to the traffic situation.
It subscribes to topics that provide information about the global trajectory, the current position and heading of the vehicle and the speed limits from the OpenDrive map to navigate safely in the simulation.
It also contains and processes services for starting and ending an overtake and the current overtake status.

The trajectory is calculated by the global planner and is adjusted by the motion planning node.
When the decision making node decides that an overtake maneuver is necessary, the motion planning node will receive this information via the services mentioned above and adjusts the trajectory accordingly.
Otherwise the received trajectory is published without any changes.

## ROS Data Interface

### Subscribed Topics

This node subscribes to the following topics:

- `/paf/hero/global_current_heading`: Subscribes to the filtered heading of the ego vehicle.
- `/paf/hero/global_current_pos`: Subscribes to the filtered position of our car.
- `/paf/hero/speed_limits_OpenDrive`: Subscribes to the OpenDrive map speed limit.
- `/paf/hero/trajectory_global`: Subscribes to the global trajectory, which is calculated at the start of the simulation.

### Published Topics

This node publishes the following topics:

- `/paf/hero/trajectory`: Publishes the global trajectory.
- `/paf/hero/trajectory_local`: Publishes the new adjusted trajectory.
- `/paf/hero/speed_limit`: Publishes the speed limit.

### Provided Services

- `/paf/hero/motion_planning/start_overtake`: Service for starting an overtake with given parameters.
- `/paf/hero/motion_planning/end_overtake`: Service for ending an overtake with given parameters.
- `/paf/hero/motion_planning/overtake_status`:  Service for the current status of an overtake.

## Node Creation + Running Tests

To run this node insert the following statement in the [planning.launch](../../code/planning/launch/planning.launch) file:

```xml
<node pkg="planning" type="motion_planning.py" name="MotionPlanning" output="screen">
           <param name="role_name" value="hero" />
           <param name="control_loop_rate" value="0.05" />
</node>
```
