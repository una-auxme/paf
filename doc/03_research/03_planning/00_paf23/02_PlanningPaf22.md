# Planning in PAF 22

[(Github)](https://github.com/ll7/paf22)

## Architecture

![overview](../../../00_assets/planning/overview.jpg)

### Preplanning

Use map and route to find path.
Without obstacles.

### Decision Making

Decides if preplanned path can be taken.
Chooses fitting action and forwards information to local path planning.

### Local path planning

Translates action into a concrete path.

## Planning documentation

### Preplanning in code

Create Trajectory out of MapData.
File: preplanning_trajectory.py

- No subscribers or publishers
- provides OpenDriveConverter class
- calculates the global trajectory

### Global Plan in code

Collects leaderboard data for preplanning.
Publishes trajectory and speed limit to other componentens (Acting, Decision Making).

File: global_planner.py

- ros-node
- Provides PrePlanner class
- methods: world_info, global_route, position

Subscribed Topics:
If one of them gets updated the corresponding method is called

- OpenDrive map
- global plan
- current agent position

Published topics:

- preplanned trajectory
- prevailing speed limits

### Decision Making in code

Makes Decision based on information from other components.
Should cover all possible traffic scenarios.

File: behaviour_tree.py, intersection.py, lane_change.py, ...

- Behaviours publish only target_speed

## Conclusion

- Difference between architecture and planning documentation
- No real local path component

### What can be done next

- Create Architecture for Local Planning
- Decide which code can be used for our project
- refactoring code (some methods are over 100 lines)
