# Planned architecture of vehicle agent

**Summary:** This page gives an overview over the planned general architecture of the vehicle agent.
The document contains an overview over all [nodes](#overview) and [topics](#topics).

---

## Authors

Samuel Kühnel

## Date

29.03.2024

---
<!-- TOC -->
- [Planned architecture of vehicle agent](#planned-architecture-of-vehicle-agent)
  - [Authors](#authors)
  - [Date](#date)
  - [Overview](#overview)
  - [Perception](#perception)
    - [Obstacle Detection and Classification](#obstacle-detection-and-classification)
    - [Traffic Light Detection](#traffic-light-detection)
    - [Localization](#localization)
  - [Planning](#planning)
    - [Global Planning](#global-planning)
    - [Decision Making](#decision-making)
    - [Local Planning](#local-planning)
      - [Collision Check](#collision-check)
      - [ACC](#acc)
      - [Motion Planning](#motion-planning)
  - [Acting](#acting)
    - [Path following](#path-following)
    - [Velocity control](#velocity-control)
    - [Vehicle controller](#vehicle-controller)
    - [Emergency](#emergency)
  - [Visualization](#visualization)
<!-- TOC -->

## Overview

The vehicle agent is split into three major components: [Perception](#Perception), [Planning](#Planning)
and [Acting](#Acting).
A separate node is responsible for the [visualization](#Visualization).
The topics published by the Carla bridge can be
found [here](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/).
The msgs necessary to control the vehicle via the Carla bridge can be
found [here](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg)

![Architecture overview](../00_assets/overview.jpg)
The miro-board can be found [here]([CarlaEgoVehicleControl.msg](https://miro.com/app/board/uXjVPC_3kvU=/)).

## Perception

The perception is responsible for the efficient conversion of raw sensor and map data into a useful
environment representation that can be used by the [Planning](#Planning) for further processing.

Further information regarding the perception can be found [here](../03_research/02_perception/Readme.md).

### Obstacle Detection and Classification

Evaluates sensor data to detect and classify objects around the ego vehicle.
Other road users and objects blocking the vehicle's path are recognized.
The node classifies objects into static and dynamic objects.
In the case of dynamic objects, an attempt is made to recognize the direction and speed of movement.

Subscriptions:

- ```radar``` ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- ```lidar``` ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- ```rgb_camera``` ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ````gnss```` ([sensor_msgs/NavSatFix](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/))
- ```map``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

Publishes:

- ```obstacles``` (Custom msg:
  obstacle ([vision_msgs/Detection3DArray Message](http://docs.ros.org/en/api/vision_msgs/html/msg/Detection3DArray.html))
  and its classification ([std_msgs/String Message](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)))

### Traffic Light Detection

Recognizes traffic lights and what they are showing at the moment.
In particular traffic lights that are relevant for the correct traffic behavior of the ego vehicle,
are recognized early and reliably.

Subscriptions:

- ```map``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```rgb_camera``` ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```lidar``` ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- ```gnss``` ([sensor_msgs/NavSatFix](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/))

Publishes:

- ```traffic_lights``` (Custom msg:
  state ([std_msgs/UInt8 Message](https://docs.ros.org/en/api/std_msgs/html/msg/UInt8.html])),
  position ([geometry_msgs/Pose Message](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)),
  distance_to_stop_line ([std_msgs/Float64 Message](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)))

### Localization

Provides corrected accurate position, direction and speed of the ego vehicle

Subscriptions:

- ```map``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```imu``` ([sensor_msgs/Imu Message](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- ```speedometer``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```gnss``` ([sensor_msgs/NavSatFix](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/))

Publishes:

- ```ego_position``` ([nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

## Planning

The planning uses the data from the [Perception](#Perception) to find a path on which the ego vehicle can safely reach
its destination. It also detects situations and reacts accordingly in traffic. It publishes signals such as a trajecotry or a target
speed to acting.

Further information regarding the planning can be found [here](../03_research/03_planning/Readme.md).

### [Global Planning](../07_planning/Global_Planner.md)

Uses information from the map and the path specified by CARLA to find a first concrete path to the next intermediate
point.

Subscriptions:

- ```map``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```navigation``` (waypoints and high-level route description)
- ```odometry``` ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

Publishes:

- ```provisional_path``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

### [Decision Making](../07_planning/Behavior_tree.md)

Checks whether the path from [Preplanning](#Preplanning) actually can be taken.
If the data from the [Perception](#Perception) indicates that the path needs to be adjusted,
this node decides which actions to take.
Based on this decision, the [Local path planning](#Local-path-planning) plans a new path accordingly.

Subscriptions:

- ```map``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```navigation``` (waypoints and high-level route description)
- ```odometry``` ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- ```provisional_path``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- all data from [Perception](#Perception)

Publishes:

- ```decision``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

### [Local Planning](../07_planning/Local_Planning.md)

It consists of three components:

- Collision Check: Checks for collisions based on objects recieved from [Perception](#perception)
- ACC: Generates a new speed based on a possible collision recieved from Collision Check and speedlimits recieved from [Global Planner](#global-planning)
- Motion Planning: Decides the target speed and modifies trajectory if signal recieved from [Decision Making](#decision-making)

#### [Collision Check](../07_planning//Collision_Check.md)

Subscriptions:

- ```Speed``` ([carla_msgs/Speedometer](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```unstuck_flag``` ([std_msgs/Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))
- ```unstuck_distance``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```speed_limits_OpenDrive``` ([std_msgs/Float32MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html))
- ```collision``` ([std_msgs/Float32MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html))
- ```trajectory_global``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```acc_velocity``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```current_wp``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```speed_limit``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

#### [ACC](../07_planning/ACC.md)

Subscriptions:

- ```Speed``` ([carla_msgs/Speedometer](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```objects``` ([std_msgs/Float32MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html))

Publishes:

- ```emergency``` ([std_msgs/Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))
- ```collision``` ([std_msgs/Float32MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html))
- ```oncoming``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

#### [Motion Planning](../07_planning/motion_planning.md)

Subscriptions:

- ```Spawn_car``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```speed_limit``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```Speed``` ([carla_msgs/Speedometer](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```current_heading``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```trajectory_global``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```current_pos``` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```curr_behavior``` ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```unchecked_emergency``` ([std_msgs/Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))
- ```acc_velocity``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```waypoint_distance``` ([perception/Waypoint](../../code/perception/msg/Waypoint.msg))
- ```lane_change_distance``` ([perception/LanecChange](../../code/perception/msg/LaneChange.msg))
- ```collision``` ([std_msgs/Float32MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html))
- ```traffic_light_y_distance``` ([std_msgs/Int16](https://docs.ros.org/en/api/std_msgs/html/msg/Int16.html))
- ```unstuck_distance``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```trajectory``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```target_velocity``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```current_wp``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```overtake_success``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

## Acting

The job of this component is to translate the trajectory planned by the [Planning](#Planning) component into
steering controls for the vehicle.
This node only takes on tasks that have lower computing time, so that a fast response of the component is ensured.

Further information regarding the acting can be found [here](../03_research/01_acting/Readme.md).

### Path following

Calculates steering angles that keep the ego vehicle on the path given by
the [Local path planning](#Local-path-planning).

Subscriptions:

- ```path``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```steer```
  for ```vehicle_control_cmd_manual``` ([CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg))

### Velocity control

Calculates velocity inputs to drive the velocity given by the [Local path planning](#Local-path-planning).
If the node is given the distance to a car to follow, it reduces the velocity of the ego vehicle to hold a reasonable
distance to the vehicle in front.

Subscriptions:

- ```ego_position``` ([nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
- ```max_velocity``` ([std_msgs/Float64 Message](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html]))
- ```distance_to_next_vehicle``` ([std_msgs/Float64 Message](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html]))
- ```path``` ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```throttle```,  ```brake```,  ```reverse```
  for ```vehicle_control_cmd_manual``` ([CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg))

### Vehicle controller

Decides which steering controller to use and assembles [CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg).

Subscriptions:

- ```throttle```,  ```brake```,  ```reverse```
- ```steer```

Publishes:

- ```vehicle_control_cmd_manual``` ([CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg))

### Emergency

Initiates emergency braking if an emergency-msg is received and notifies all notes when emergency braking is over.

Subscriptions:

- ```emergency``` ([std_msgs/Bool Message](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

Publishes:

- ```vehicle_control_cmd_manual``` ([CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg))
- ```emergency``` ([std_msgs/Bool Message](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

## Visualization

Visualizes outputs of certain nodes to provide a basis for debugging.

Subscriptions:

- all data that's needed