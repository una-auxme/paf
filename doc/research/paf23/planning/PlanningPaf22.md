# Planning in PAF 22

**Summary:** This page contains research into the planning component of the PAF22 group.

[(Github)](https://github.com/ll7/paf22)

- [Planning in PAF 22](#planning-in-paf-22)
  - [Architecture](#architecture)
    - [Preplanning](#preplanning)
    - [Decision Making](#decision-making)
    - [Local path planning](#local-path-planning)
  - [Planning documentation](#planning-documentation)
    - [Preplanning in code](#preplanning-in-code)
    - [Global Plan in code](#global-plan-in-code)
    - [Decision Making in code](#decision-making-in-code)
  - [Conclusion](#conclusion)
    - [What can be done next](#what-can-be-done-next)

## Architecture

![overview](../../../assets/planning/overview.jpg)

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
