# Potential Field Method for trajectory refinement

**Summary:** This page informs about the potential field local planner which has been discontinued

## Table of contents

- [Table of contents](#table-of-contents)
- [Why it was discontinued](#why-it-was-discontinued)
- [Overview](#overview)
- [Concept of the Potential field local planner](#concept-of-the-potential-field-local-planner)
- [Current state](#current-state)
- [Improvement suggestions](#improvement-suggestions)
- [Further integration Problems](#further-integration-problems)
- [Sources](#sources)

## Why it was discontinued

The Development of the Potential Field method as a local path planner was discontinued due to:

- Difficult tuning due to the large parameterset
- Performance issues originating in the high number of array calculations used in it
- Bad GPS-localisation resulting in jumping of the global trajectory relative to the hero position
- transformation problems from the global trajectory to the hero frame

## Overview

The potential Field method was developed to provide a robust path planner in uncertain scenarios and scenarios where quick, simple decision-making was important (e.G the mini car door, when stuck etc.)

## Concept of the Potential field local planner

The thoughtprocess leading to the potential field planner was to provide a very robust path finding its way through every entity on the global path by giving them a repulsive potential. By doing this, we can use gradient descent so generate a path from high potential (meaning close to obstacles)
to low potential (meaning away from obstacles or in a safe drivable area). When this step was done, the problem arose, that the planned trajectory didn't match up to the direction where the next point
on the global trajectory lied.
Since we want the car to move forward in order to complete its mission, a slope was introduced into the entity matrix, giving the whole potential field a tunable gradient to the direction the car was facing. Now the trajectory generated is attracted to the front edge of the
so called *Potential field horizon* (the area in which the trajectory is computed around the entities).
Since the global trajectory has turns at intersection that have to be followed, this version of the potential field did not suffice. The fix is to transform the global trajectory to the "hero" coordinate frame. Afterwards, this trajectory was layered onto the potential
field by subtracting the attractive potential from the potential field in order to create a force drawing the trajectory to the global trajectory (think about a waterslide guiding the trajectory).

## Current state

The potential field trajectory currently is able to provide a trajectory as presented in [Concept of the Potential field local planner](#concept-of-the-potential-field-local-planner). It was not used because its unreliability, high computing times and difficult integration into the existing controllers.
Sometimes the trajectory gets stuck in the hero vehicle and does not converge because of the hero vehicle not getting filtered out correctly in the intermediate layer.
Another problem is presented by transforming the global trajectory to the hero frame. The provided libraries by ros (tf, tf2, ...) are not easy to work with. Therefore, the transformation has been calculated without the use of these libraries which likely introduces errors.
Another drawback is the lack of visualization of the potential in rviz. Visualization with markers
didn't work in the limited timeframe although, it would have been helpful for debugging and reviewing the result of the calculations of the node. This is likely again due to the coordinate transformation.
Despite its drawbacks, some successes have been recorded, in which the potential field generated a trajectory leading the car out of a difficult situation back onto its path.
It showed that the potential field method can be utilized to plan a path based on very little information (only subscribes the intermediate layer and the global trajectory).

## Improvement suggestions

- Increase performance by speeding up the matrix calculations with tools like cython.
- Counter the fickle behavior of the trajectory by giving the trajectory some form of inertia from one calculation step to another
- Merge the entities into a shapely multipolygon then extract only the bounding lines to incorporate more information but not more than needed
  (from a shapely multipolygon to a numpy array can be done by extracting the start and endpoints in the lines of the polygon and then "drawing" them onto an array with cv2.line for example).
- Handle the lane markings included in the entity list right now separately in order to enable weighing them differently to the car entities.
- Refactor the entire node, the potential field development was a lot of trial and error so a lot of the functions and their calls are not in the place they need to be.

## Further integration Problems

- in order to integrate the new trajectory the controllers have to be adjusted.
  - the stanley controller calculates the position error of the hero vehicle from the trajectory.
    since the trajectory origin is in the hero position, this error should be 0 and if not, its because of gps errors and not a path following error. This makes the stanley controller entirely not usable with the potential field method (in this state)
  - the pure pursuit controller is suitable for use with this trajectory, but another problem arises when trying to use it: The pure pursuit controller (and the stanley controller) need to determine a suitable point on the trajectory as a temporary goal point.
  This is done by working with the index of the point in the point/pose list of the path message the car is currently at, which - in the case of the potential field trajectory - is always 0. Integrating the controller as it is into the potential field trajectory following did not work out of the box.
- The ACC currently looks at the curve radius of the global trajectory to determine the speed with which the corner should be driven. The ACC will have to be adjusted in order to use the new trajectory
- Due to the fickle behavior of the trajectory the steering angle in the controllers will be very fickle aswell which can lead to unexpected behavior of the car as a result.

## Sources

Here I will provide some sources needed to continue the development of the Potential field trajectory planning

- [Lecture about Potential fields](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)
- [the tf module in ros](http://wiki.ros.org/tf) (for coordinate transformations, a bit hard to work with)
- [shapely multipolygon class](https://shapely.readthedocs.io/en/latest/reference/shapely.MultiPolygon.html) (the [entities](../../code/mapping/ext_modules/mapping_common/entity.py) in the intermediate layer can be transformed to shapely polygons)
- [cython](https://cython.org/) for speeding up the calculations
- [potential field node](./discontinued/potential_field_node.py) (place [here](../../code/acting/src/acting/) to continue work)
- [the map class](../../code/mapping/ext_modules/mapping_common/map.py)