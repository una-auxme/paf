# Dev round - Sprint 4

**Summary:** For better communication a developer round is held in this sprint to improve communication and clarify responsibilities.

---

## Author

Josef Kircher

Julian Graf

Marco Riedenauer

## Date

01.02.2023

## Prerequisite

---
<!-- TOC -->
* [Dev round - Sprint 4](#dev-round---sprint-4)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Planning](#planning)
    * [Things that need to handled](#things-that-need-to-handled)
    * [Results](#results)
  * [Acting](#acting)
  * [Perception](#perception)
    * [Sources](#sources)
<!-- TOC -->

## Planning

### Things that need to handled

* Information from perception
  * What will the traffic light message look like?
  * How will traffic sign information be published?
  * Can lidar detect obstacles and assign them to a lane?
  * Is lane detection handled by your team or should it be done by the map analysis?
  * Doors detected?

* Information from acting
  * When is ACC used, who should it activate/deactivate?
  * What else do you need from planning besides a path and a velocity?
  * What should the path look like, how dense should it be?
  * How should we handle re-planning of the path? New message?

### Results

* Acting
  * ACC controlled by decision tree
  * Possibility of ACC to work with static obstacles
  * two topics for target speed. Perception publishes the street limits to one topic, planning publishes an artificial target speed e.g. at intersections to another topic. Acting takes the minimal value. Planning send a predefined value to give priority to street limit speed
  * path points can be as far away from each other as we want as long a linear interpolation is feasible.
  * New message for new path is the way to go, regular updates for the path would be nice

* Perception
  * Traffic light: status, distance, (lane?)
  * Radar gives a relative speed of other vehicles at every tick
  * traffic sign: state, distance
  * lane detection will be most likely handled by perception

## Acting

no questions so far

## Perception

no questions so far

### Sources
