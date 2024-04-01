# Planning Implementation

**Summary:**
The document gives a first impression of how the planning could/should work
and how the topics are edited

---

## Authors

Simon Erlbacher, Niklas Vogel

## Date

29.11.2022

---
<!-- TOC -->
* [Planning Implementation](#planning-implementation)
  * [Authors](#authors)
  * [Date](#date)
  * [Overview](#overview)
  * [Preplanning](#preplanning)
  * [Decision Making](#decision-making)
  * [Local Path Planning](#local-path-planning)
  * [Next steps](#next-steps)
* [Sources](#sources)
<!-- TOC -->

---

## Overview

![Implementation](../../00_assets/Planning_Implementierung.png)
[Link to original](https://miro.com/app/board/uXjVP_LIQpE=/?share_link_id=806357474480)

---

## Preplanning
  
The Preplanning receives the next [target point](./basics.md#wie-sehen-die-daten-vom-leaderboard-für-das-global-planning-aus) from the Carla Leaderboard. Also it reads the map data which is in [OpenDrive](https://www.asam.net/standards/detail/opendrive/) format.
The [commonroad route planner (CRP)](https://gitlab.lrz.de/tum-cps/commonroad-route-planner/) from TUM can be used to calculate a Lanelet model. Note that almost all old PAF-Projects have adapted versions of the CRP. Internally the CRP converts the Map into a Scenario Object.
When given a start and goal position and the current orientation of the vehicle, a so called planning problem is created by the planer.
Either you use the given waypoints for start and goal values or alternatively the orientation can also be calculated with the odometry and the GNUU data from the sensoring area.
The Output (Solution of the planning problem) will be a route defined by a sequence of lanelets and a sequence of points (~ 10cm apart).

Lanelet Model Example :
![Lanelet Model Example](../../00_assets/Lanelets.png)
[(Source)](https://github.com/ll7/psaf2/tree/main/Planning/global_planner)

Input:

* Map
* Navigation Waypoints
* (Odometry data (sensoring))
* (GNUU data (sensoring))

Output:

* Route (Sequences of Lanelets and Points) (local path planning, decision making)

---

## Decision Making

If an obstacle, which interferes with the own trajectory, is being recognized in the [perception](../02_perception),
the decision making sends a message to the local path planning where the system then chooses another trajectory/lanelet.
With the Lanelets Model it is easier to give a prediction for other objects and the vehicle itself,
by following the lane direction of an object. With the prediction, which is mainly based inside the [perception](../02_perception),
it's then possible to check weather or not other objects interfere with ourselves.

The decision making can be implemented with a state machine. Therefore there must be a state defined for every incoming perception/situation to ensure correct and save behavior.
The system needs to make good predictions to avoid collisions. The Perception data and the Lanelet modell are the main input for the system.

Input:

* Lanelet data (preplanning, local path planning)
* perception data (traffic lights situation, pedestrians,...)

Output:

* updated driving status (acting, local path planning)
* Lanelet data (acting)

---

## Local Path Planning

Local Planner updates the current route, if Decision Making detects an obstacle. The planner then has to choose an alternative next Lanelet to avoid the obstacle. Therefore the obstacle position on the lanelet plan will be needed.

### Velocity profile

The Local Path Planer receives the lanelets, points and the path to drive.
The local planner creates a velocity profile on the calculated trajectory based on curvature, crossings and traffic lights.
This will be calculated directly after the preplanning created a trajectory. The velocity value is published to the [acting side](../01_acting).

Input:

* Trajectory points (preplanning)

Output:

* Max. Velocity (Acting)

### Update path

The update path module receives a command from the decision maker, that the current path is not possible to drive without collision.
If necessary it uses the lanelet modell from the preplanner and chooses the next possible lane and accordingly updates the trajectory.
Sometimes it might not be necessary to change lane completely when a little adjustment of the trajectory inside the lane is sufficient.
It also tells the velocity profile to update because of the new trajectory.

Input:

* lanelet modell (preplanning)
* update command (decision making)
* information about blocked lanelets (decision making, perception)

Output:

* updated trajectory (acting, decision making)
* update command (velocity profile)

### Measure distance

This module measures the distance to obstacles, especially cars, with the Lidar Sensor. The current distance value is published to the [acting side](../01_acting) for keeping a safe distance (Adaptive Cruise Control).

Input:

* Lidar Sensor data (perception, sensoring)

Output:

* distance value (acting)

---

## Next steps

* Another Coordination with Perception to prevent overlaps with Map Manager, Map enrichment,
* Implement Map Manager to convert data into a compatible type for route planning and to extract additional informations (Speed Limits, trafic signs, traffic lights)
* Implement a commonroad route planner (old projects and Gitlab TUM)
* Analyze Lanelet plan and be familiar with it (Which information can we additionally receive from the plan?)
* Enrich Lanelet Modell/Map with additional Informations (additional/parallel Lanes, Speed Limits, trafic signs, traffic lights)
* Choose the Decision Maker (Evaluate Markov Modell in combination with occupancy grid)
* calculate and evaluate distances with given perceptions
* Publish available and needed data (data available in this stage)

---

### Sources

<https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_planning>

<https://gitlab.lrz.de/tum-cps/commonroad-route-planner>
