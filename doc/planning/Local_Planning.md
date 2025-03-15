# Local Planning

**Summary:** This page contains the conceptual and theoretical explanations for the Local Planning component. For more technical documentation have a look at the other linked documentation files.

- [Overview](#overview)
- [ACC](#acc)
- [Motion Planning](#motion-planning)
  - [Cornering Speed](#cornering-speed)
  - [Selecting the target velocity](#selecting-the-target-velocity)
- [Sources](#sources)

## Overview

The Local Planning component is responsible for evaluating short term decisions in the local environment of the ego vehicle. Some examples can be collision avoidance, reducing speed or emergency brakes.

The Local Planning in this project is divided in two components: Adaptive Cruise Control (ACC) and Motion Planning.

The architecture can be seen below:

![Planning_architecture.png](../assets/planning/Planning_architecture.png)

The theoretical concepts of each Local Planning component are explained below.

## [ACC](./ACC.md)

The ACC is responsible for evaluating the target speed based on the current traffic situation. It recieves the speed limits from the global planner and the collisions from the Collision Check.
The ACC permanently evaluates in an infinite loop if a collision is recieved and calculates a new target speed accordingly.

**If a collision is recieved**: Caluclate the distance to stop at the current speed without the emergency brake as safety distance that should be kept.

$$ d_{safety} = t_{reaction} \cdot v[\frac{m}{s}] + (\frac{v[\frac{km}{h}]}{10})^2 $$

**If distance to object < braking distance**: Calculate new target speed.

$$ v_{target} = v_{ego} \cdot \frac{d_{current}}{d_{safety}}$$

With this formula we can ensure that the target speeds shrinks in relation on how big the difference between current distance to object and desired safety distance is.

**If distance to object $\geq$ braking distance**: Keep current speed

**If no collision recieved**: Drive with speed limit.

The target speed is published to Motion Planning that evaluates what speed should be driven also with respect to the current behavior.

## Motion Planning

The Motion Planning is the central control of the Local Planning. Controlling the target velocity and the trajectory to be executed in the acting. Following features are implemented:

### Cornering Speed

![Corner Speed - Full Trajectory.png](../assets/planning/plot_full_trajectory_1_degree.png)

The cornering speed gets calculated at the beginning of the scenario, when the full trajectory is received:

- Calculate the angle of three points in the trajectory with atan2
- Find indices with a difference of 1°
- Get speed according to the length of the corner

Lane changes are special, because you can drive the with normal speed eventhough they have a curve change (visible in the picture on the straights). They can get their own speed, because every lane change is way smaller than the smallest corner.

### Selecting the target velocity

The target velocity is a combination of the acc speed, the behavior speed and the cornering speed. Almost everytime the minimal speed is choosen. Exceptions are overtaking and the parking maneuver.

![Scenario](../assets/planning/three_scenarios.png)

In the first scenario on the left side the green ego vehicle chooses the acc speed to not cause a collision with the red car.
In the second scenario the car is waiting at the intersection and chooses the behavior speed (wait at intersection), while the acc would say speedlimit.
In the last scenario the car chooses the cornering speed to smoothly perform a 90° right turn.

## Sources

<https://www.adac.de/verkehr/rund-um-den-fuehrerschein/erwerb/anhalteweg-berechnen/>
