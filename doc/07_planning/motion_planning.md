# Motion Planning

**Summary:** [motion_planning.py](.../code/planning/local_planner/src/motion_planning.py):
The motion planning is responsible for collecting all the speeds from the different components and choosing the optimal one to be fowarded into the acting.

---

## Author

Julius Miller

## Date

17.12.2023

## Prerequisite

---
<!-- TOC -->
- [Motion Planning](#motion-planning)
  - [Author](#author)
  - [Date](#date)
  - [Prerequisite](#prerequisite)
  - [Description](#description)
    - [Inputs](#inputs)
    - [Outputs](#outputs)
<!-- TOC -->

---

## Description

This node currently gathers the behavior speed and the acc_speed and chooses the lower one to be forwarded to the acting.
At the moment this is enough since the only present behaviors are Intersection, Lane Change and Cruise.

When the Overtaking behavior will be added, choosing the minimum speed will not be sufficient.

This file is also responsible for providing a ```target_speed of -3``` for acting, whenever we need to use the Unstuck Behavior. -3 is the only case we can drive backwards right now,
since we only need it for the unstuck routine. It also creates an overtake trajectory, whenever the unstuck behavior calls for it.

### Inputs

This node subscribes to the following topics:

- Current Behavior:
  - `/paf/{role_name}/curr_behavior` (String)
- ACC Velocity:
  - `/paf/{role_name}/acc_velocity` (Float32)
- Waypoint:
  - `/paf/{role_name}/waypoint_distance` (Waypoint)
- Lane Change:
  - `/paf/{role_name}/lane_change_distance` (LaneChange)

### Outputs

This node publishes the following topics:

- Target Velocity
  - `/paf/{role_name}/target_velocity` (Float32)
