# Requirements and challenges for an acting implementation

**Summary:** On this page you can find the results of the basic research on acting summed up into resulting requirements and function, that were already agreed upon.

---

## Authors

Gabriel Schwald

### Date

20.11.2022

---

<!-- TOC -->
* [Requirements and challenges for an acting implementation](#requirements-and-challenges-for-an-acting-implementation)
  * [Authors](#authors)
    * [Date](#date)
  * [Planned basic implementation of the Acting domain](#planned-basic-implementation-of-the-acting-domain)
  * [List of basic functions](#list-of-basic-functions)
  * [List of Inputs/Outputs](#list-of-inputsoutputs)
  * [Challenges](#challenges)
  * [Next steps](#next-steps)
<!-- TOC -->

This document sums up all functions already agreed upon in [#24](https://github.com/ll7/paf22/issues/24) regarding [acting](../01_acting/01_acting.md), that could be implemented in the next sprint.

## Planned basic implementation of the Acting domain

The basic goals of the acting domain can be summed up as follows:
The vehicle must know it's own _position_, _heading_ and _velocity_.
With the addition of a provided _trajectory_ it is then able to provide _throttle_ and _steering angle_ outputs.
These goals lead to the following requirements:

## List of basic functions

* Longitudinal control
  * PID controller
* Lateral control
  * Pure Pursuit controller
  * Stanley controller

## List of Inputs/Outputs

* Subscribes to:
  * [nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) : to get the current position and heading
  * [nav_msgs/Path Message](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) : to get the current trajectory
  * emergency breaking msg : to initiate emergency breaking
  * speed limit msg : to get the maximum velocity
* Publishes:
  * [CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/#carlaegovehiclecontrolmsg) : to actually control the vehicles throttle, steering, ...

## Challenges

A short list of challenges for the implementation of a basic acting domain and how they these could be tackled based on the requirements mentioned above.

* The vehicle needs to know its own position => [nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) or [GNSS](https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor) sensor
* The vehicle needs to know its own velocity => can be calculated from last/current position and time or the [speedometer](https://leaderboard.carla.org/#map-track) pseudosensor can be used
* The vehicle needs to know its planned trajectory => [nav_msgs/Path Message](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) this trajectory may need to be updated to accommodate obstacles
* Longitudinal control => a simple PID controller should suffice
* lateral control => Pure Pursuit as well as Stanley controller should be implemented, following tests can show, where to use each controller.
* additional features:
  * emergency breaking => this command is supposed to bypass longitudinal and lateral controllers (and should use the bug discoverd by [paf21-2](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#bugabuses))
  * additional functionality mostly should be added here ...

## Next steps

Following this description of features and requirements, work on coding those features could immediately start for next weeks sprint (new Issue). This code could in most aspects closely ressemble the work done by last years group [paf21-1/../vehicle_control](https://github.com/ll7/paf21-1/tree/master/components/vehicle_control/node/src/vehicle_control/driving).
After [#33](https://github.com/ll7/paf22/issues/33) is resolved, additional  resulting features should be looked at (new Issue).
