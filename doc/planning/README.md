# Documentation of Planning

## Architecture Diagram

**TO DO**: Add snippet from the global architecture diagram.

## Overview

### Global Planner (/global_planner)

#### [Preplanning / OpenDrive Converter (preplanning_trajectory.py)](./Preplanning.md)

This module focuses on creating a trajectory out of
an OpenDrive map (ASAM OpenDrive). As input it receives an xodr file (OpenDrive format) and the target points
from the leaderboard with the belonging actions. For example action number 3 means, drive through the intersection.

#### [Global Planner (global_planner_node.py)](./Global_Planner.md)

The global planner is responsible for collecting and preparing all data from the leaderboard and other internal
components that is needed for the preplanning component.
After finishing that this node initiates the calculation of a trajectory based on the OpenDriveConverter
from preplanning_trajectory.py. In the end the computed trajectory and prevailing speed limits are published.

![img.png](../assets/Global_Plan.png)

#### Global Planner Distance Publisher (global_plan_distance_publisher.py)

The global planner distance publisher is responsible for publishing distance, position and type of the next waypoint.
The decision making uses this information for triggering special events (e.g. lanechange left ahead, intersection turn right).

### [Local Planner (/local_planner)](./Local_Planning.md)

This module includes the Nodes: \
[ACC (ACC.py)](./ACC.md) and [MotionPlanning (motion_planning.py)](./motion_planning.md)

The Local Planning package is responsible for evaluating short term decisions in the local environment of the ego vehicle. It contains components responsible for detecting collisions and reacting e. g. lowering speed.
The local planning also executes behaviors e.g. changes the trajectory for an overtake.

![Overtake](../assets/planning/Overtake_car_trajectory.png)

### [Decision making (/behavior_agent)](./Behavior_tree.md)

This module includes the Nodes: BehaviorTree and its subbehaviors

The decision making collects most of the available information of the other components and makes decisions based on
the information. All possible traffic scenarios are covered in this component. The decision making uses a so called
decision tree, which is easy to adapt and to expand.

![Simple Tree](../assets/planning/behaviour_tree.PNG)
