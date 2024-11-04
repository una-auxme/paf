# Relation beteween local and global planning

Communication between Global and Local Planning is done via ros topics. Here is a graph showing the relation between the nodes.

- Global Planning: PrePlanner
- Local Planning: ACC (Adaptive Cruise Control), CollisionCheck, MotionPlanning

![Graph relation](/doc/assets/research_assets/planning_internal.png)

## Global Planning

Global planning consists of the PrePlanner Node and the OpenDriveConverter.

- OpenDriveConverter: The class is primarily used to process OpenDrive maps and extract relevant information needed for trajectory planning.
- PrePlanner Node: The PrePlanner node is responsible for creating a trajectory out of an OpenDrive Map with the belonging road options.
  - the `/paf/hero/trajectory_global` topic published is used internally for the preplanned trajectory.

## Local Planning

Local planning consists of the ACC, CollisionCheck and MotionPlanning nodes. It uses information from the Global Planning to make decisions in the local environment of the ego vehicle.
It creates a trajectory based on the global trajectory and the current position of the vehicle that is used by the acting component.

- ACC: The Adaptive Cruise Control node is responsible for calculating the desired speed of the vehicle based on the current speed, trajectory, and speed limits.
  - receives Speed from `/carla/hero/Speed`
  - receives speed limits from `/paf/hero/speed_limits_OpenDrive`
  - receives global trajectory from `/paf/hero/trajectory_global`
- CollisionCheck: The Collision Check node is responsible for detecting collisions and reporting them.
  - receives current speed from `/carla/hero/Speed`
  - receives distances to objects detected by a LIDAR sensor from `/paf/hero/Center/object_distance`
- MotionPlanning: The Motion Planning node is responsible for calculating the desired trajectory based on the current position of the vehicle and the global trajectory.
  - receives speed information from ACC
  - receives global trajectory from `/paf/hero/trajectory_global`
  