# Relation beteween local and global planning

Communication between Global and Local Planning is done via ros topics. Here is a graph showing the relation between the nodes.

- Global Planning: PrePlanner
- Local Planning: ACC (Adaptive Cruise Control), CollisionCheck, MotionPlanning

![Graph relation](/doc/assets/research_assets/planning_internal.png)

## Global Planning

Global planning consits of the PrePlanner Node and the OpenDriveConverter.

- OpenDriveConverter: The class is primarily used to process OpenDrive maps and extract relevant information needed for trajectory planning.
- PrePlanner Node: The PrePlanner node is responsible for creating a trajectory out of an OpenDrive Map with the belonging road options.
  - the `/paf/hero/trajectory_global` topic published is used internally for the preplanned trajectory.

## Local Planning

