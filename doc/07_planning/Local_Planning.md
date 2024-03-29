# Local Planning

**Summary:** This page contains the conceptual and theoretical explanations for the Local Planning component. For more technical documentation have a look at the other linked documentation files.

---

## Author

Samuel Kühnel

## Date

29.03.2024

## Prerequisite

---
<!-- TOC -->
- [Local Planning](#local-planning)
  - [Author](#author)
  - [Date](#date)
  - [Prerequisite](#prerequisite)
  - [Overview](#overview)
  - [Collision Check](#collision-check)
    - [Apply filters](#apply-filters)
    - [Calculate Speed](#calculate-speed)
    - [Check for collision](#check-for-collision)
    - [ACC](#acc)
  - [Motion Planning](#motion-planning)
    - [Sources](#sources)
<!-- TOC -->
## Overview

The Local Planning component is responsible for evaluating short term decisions in the local environment of the ego vehicle. Some examples can be collision avoidance, reducing speed or emergency brakes.

The Local Planning in this project is divided in three components. Collision Check, Adaptive Cruise Control (ACC) and Motion Planning. The architecture can be seen below:

![Planning_architecture.png](../00_assets/planning/Planning_architecture.png)

The theoretical concepts of each Local Planning component are explained below.

## [Collision Check](./Collision_Check.md)

The Collision Check is the backbone of the Local Planning. Its task is to detect collisions with objects published by the vision node. The workflow when new objects are recieved looks like this:

![collision_check.png](../00_assets/planning/collision_check.png)

### Apply filters

The following input is recieved by the perception: $[class, min⁡(𝑎𝑏𝑠(𝑦)), min⁡(𝑥)]$ in a $(nx3)$-matrix
Filtering steps:

![vision_objects_filter_cc.png](../00_assets/planning/vision_objects_filter_cc.png)

We filter for the following traffic objects: Pedestrians, bicycles, bikes, cars, busses and trucks. To filter oncoming traffic the $y$-distance is used as a deviation from the cars's middle axis (+ left, - right).

### Calculate Speed

As we are not able to use the radar sensor, we need to approximate the objects's speed to determine a collision. For this we simply save the last to distances recieved and calculate the speed difference between our ego vehicle and a possible collision object.

Lets have a look at the following example: $d_1 = 20, d_2 = 19, \Delta t = 0.5s, v_{ego} = 10 \frac{m}{s}$

- We saved the last to distances and the time difference between them. So now we can calculate the speed difference: $\Delta v = \frac{d_2 - d_1}{0,5} = -2 \frac{m}{s}$
- To calculate the absolute speed we can add the difference to our own speed: $v_{approx} = v_{ego} + \Delta v = 8 \frac{m}{s}$

### Check for collision

How to determine if a collision is ahead? --> Simple decision: If our speed > object speed we will collide sooner or later.

**If our speed > object speed**: First check for emergency brake!

Calculate emergency brake distance with simple thumb rule (see sources for more):

$$d_{emergency} = t_{reaction} \cdot v[\frac{m}{s}] + \frac{(\frac{v[\frac{km}{h}]}{10})^2}{2}$$

The emergency distance is composed of reaction distance determined by the reaction time of 1s and the braking distance which is approximated by converting our speed in km/h. As we check for a emergency brake our braking distance can be divided by 2, because we brake extra fast.

**If our current distance < emergency distance**: Publish emergency brake signal!

Nevertheless publish the collision information (object distance and speed) to ACC and Decision Making to trigger possible reactions.

### [ACC](./ACC.md)

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

### Sources

<https://www.adac.de/verkehr/rund-um-den-fuehrerschein/erwerb/anhalteweg-berechnen/>
