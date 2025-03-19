# Local Planning

**Summary:** This page contains the conceptual and theoretical explanations for the Local Planning component. For more technical documentation have a look at the other linked documentation files.

- [Overview](#overview)
- [Adaptive Cruise Control (ACC)](#adaptive-cruise-control-acc)
- [Motion Planning](#motion-planning)

## Overview

The Local Planning component is responsible for evaluating short term decisions in the local environment of the ego vehicle. Some examples can be collision avoidance, reducing speed or emergency brakes.

The Local Planning in this project is divided in two components: Adaptive Cruise Control (ACC) and Motion Planning.

The architecture can be seen below:

![Planning_architecture.png](../assets/planning/Planning_architecture.png)

The theoretical concepts of each Local Planning component are listed below.

## Adaptive Cruise Control (ACC)

The Adaptive Cruise Control (ACC) is the only component that decides at which speed the vehicle should drive. It uses mainly the distance to and the speed of the leading vehicle to calculate a reasonable speed. For further and more detailed information, check the [ACC documentation](./ACC.md).

## Motion Planning

The Motion planning is responsible for adjusting and publishing the speed limit, the trajectory and the local trajectory according to the traffic situation. For further and more detailed information, check the [Motion Planning documentation](./motion_planning.md).
