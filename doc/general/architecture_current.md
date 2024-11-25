# Current architecture of the vehicle agent

**Summary:** This page gives an overview over the current general architecture of the vehicle agent.
The document contains an overview over all [nodes](#overview) and [topics](#topics). Additionally a visual thematic grouping of the subsystems of the agent is done for an easier optical identification which nodes and topics belong/contribute to them.

- [Overview](#overview)
- [Perception](#perception)
  - [Vision Node (vision\_node.py)](#vision-node-vision_nodepy)
  - [Traffic Light Detection (traffic\_light\_node.py)](#traffic-light-detection-traffic_light_nodepy)
  - [Position Heading Node (position\_heading\_publisher\_node.py)](#position-heading-node-position_heading_publisher_nodepy)
  - [Global distances (global\_plan\_distance\_publisher.py)](#global-distances-global_plan_distance_publisherpy)
  - [Kalman filtering (kalman\_filter.py)](#kalman-filtering-kalman_filterpy)
  - [Localization](#localization)
- [Planning](#planning)
  - [PrePlaner (global\_planner.py)](#preplaner-global_plannerpy)
  - [Behavior Agent (behavior\_agent)](#behavior-agent-behavior_agent)
  - [Local Planning](#local-planning)
- [Acting](#acting)
  - [MainFramePublisher](#mainframepublisher)
  - [pure\_pursuit\_controller](#pure_pursuit_controller)
  - [stanley\_controller](#stanley_controller)
  - [vehicle\_controller](#vehicle_controller)
  - [velocity\_controller](#velocity_controller)

## Overview

The vehicle agent is split into three major components: [Perception](#Perception), [Planning](#Planning)
and [Acting](#Acting).
A separate node is responsible for the [visualization](#Visualization).
The topics published by the Carla bridge can be
found [here](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/).\
The messages necessary to control the vehicle via the Carla bridge can be
found [here](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg).

The miro-board can be found [here](https://miro.com/welcomeonboard/a1F0d1dya2FneWNtbVk4cTBDU1NiN3RiZUIxdGhHNzJBdk5aS3N4VmdBM0R5c2Z1VXZIUUN4SkkwNHpuWlk2ZXwzNDU4NzY0NTMwNjYwNzAyODIzfDI=?share_link_id=785020837509).
![Architecture overview](../assets/overview.jpg)*Connections between nodes visualized*

![Department node overview](../assets/research_assets/node_path_ros.png)*In- and outgoing topics for every node of the departments*

## Perception

The perception is responsible for the efficient conversion of raw sensor and map data into a useful
environment representation that can be used by the [Planning](#Planning) for further processing.
It provides localization of the agent, filtering noise from sensor data, preconditions images for further usage in other nodes and detects certain points of interest (at the moment only traffic lights).

Further information regarding the perception can be found [here](../perception/README.md).
Research for the perception can be found [here](../research/paf24/perception/).

In the following, all subscribed and published topics and their corresponding nodes are listed with the format:

- ```name of topic``` \(origin/destination node\) (typo of message) (link to ROS documentation of the message type)

### Vision Node ([vision_node.py](/../paf/code/perception/src/vision_node.py))

Evaluates sensor data to detect and classify objects around the ego vehicle.
Other road users and objects blocking the vehicle's path are recognized (most of the time).
Recognized traffic lights get cut out from the image and made available for further processing.

Subscriptions:

- ```/paf/hero/Center/dist_array``` \(/lidar_distance\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Center/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Back/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Left/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Right/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/paf/hero/Center/segmented_image``` \(/rviz\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Back/segmented_image``` \(/rviz\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Left/segmented_image``` \(/rviz\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Right/segmented_image``` \(/rviz\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Center/object_distance``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/Back/object_distance``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/Left/object_distance``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/Right/object_distance``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/Center/segmented_traffic_light``` \(/TrafficLightNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Back/segmented_traffic_light``` \(/TrafficLightNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Left/segmented_traffic_light``` \(/TrafficLightNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Right/segmented_traffic_light``` \(/TrafficLightNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

### Traffic Light Detection ([traffic_light_node.py](/../paf/code/perception/src/traffic_light_node.py))

Recognizes traffic lights and what they are showing at the moment.
In particular traffic lights that are relevant for the correct traffic behavior of the ego vehicle,
are recognized early and reliably.

Subscriptions:

- ```/paf/hero/Center/segmented_traffic_light``` \(/VisionNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/paf/hero/Center/traffic_light_state``` \(/behavior_agent\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))
- ```/paf/hero/Center/traffic_light_y_distance``` \(/behavior_agent\) ([std_msgs/Int16](https://docs.ros.org/en/api/std_msgs/html/msg/Int16.html))

### Position Heading Node ([position_heading_publisher_node.py](/../paf/code/perception/src/position_heading_publisher_node.py))

Calculates the current_pos (Location of the car) and current_heading (Orientation of the car around the Z- axis).

Subscriptions:

- ```/carla/hero/OpenDRIVE``` \(/carla_ros_bridge\) ([sensor_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/carla/hero/IMU``` \(/carla_ros_bridge\) ([sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))
- ```/carla/hero/GPS``` \(/carla_ros_bridge\) ([sensor_msgs/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))
- ```/paf/hero/kalman_pos``` \(/kalman_filter_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/kalman_heading``` \(/kalman_filter_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/paf/hero/unfiltered_heading``` \(no subscriber at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/unfiltered_pos``` \(/kalman_filter_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_pos``` \(/pure_pursuit_controller, /stanley_controller, /MotionPlanning, /ACC, /MainFramePublisher, /GlobalPlanDistance, /position_heading_publisher_node, /curr_behavior \) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_heading``` \(/stanley_controller, /behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

### Global distances ([global_plan_distance_publisher.py](/../paf/code/perception/src/global_plan_distance_publisher.py))

Subscriptions:

- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/carla/hero/global_plan``` \(/ \) ([ros_carla_msgs/msg/CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))

Publishes:

- ```/paf/hero/waypoint_distance``` \(/MotionPlanning, /behavior_agent\) ([geographic_msgs/WayPoint](https://docs.ros.org/en/melodic/api/geographic_msgs/html/msg/WayPoint.html))
- ```/paf/hero/lane_change_distance``` \(/MotionPlanning, /behavior_agent\) ([perception/msg/LaneChange](/../paf/code/perception/msg/LaneChange.msg))

### Kalman filtering ([kalman_filter.py](/../paf/code/perception/src/kalman_filter.py))

Subscriptions:

- ```/carla/hero/OpenDRIVE``` \(/carla_ros_bridge\) ([sensor_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/carla/hero/IMU``` \(/carla_ros_bridge\) ([sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))
- ```/carla/hero/GPS``` \(/carla_ros_bridge\) ([sensor_msgs/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))
- ```/paf/hero/unfiltered_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

Publishes:

- ```/paf/hero/kalman_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/kalman_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

### Localization

Provides corrected accurate position, direction and speed of the ego vehicle. For this to be achived the

- [Position Heading Node](#position-heading-node-position_heading_publisher_nodepy)
- [Kalman filtering](#kalman-filtering) and
- [Coordinate Transformation](/../paf/code/perception/src/coordinate_transformation.py)

classes are used.

## Planning

The planning uses the data from the [Perception](#Perception) to find a path on which the ego vehicle can safely reach its destination. It also detects situations and reacts accordingly in traffic. It publishes signals such as a trajecotry or a target speed to acting.

Further information regarding the planning can be found [here](../planning/README.md).
Research for the planning can be found [here](../research/planning/README.md).

### PrePlaner ([global_planner.py](/../paf/code/planning/src/global_planner/global_planner.py))

Uses information from the map and the path specified by CARLA to find a first concrete path to the next intermediate point. More about it can be found [here](../planning/Global_Planner.md).

Subscriptions:

- ```/carla/hero/OpenDRIVE``` \(/carla_ros_bridge\) ([sensor_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/carla/hero/global_plan``` \(/ \) ([ros_carla_msgs/msg/CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

Publishes:

- ```/paf/hero/trajectory_global``` \(/ACC, /MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/speed_limits_OpenDrive``` \(/ACC\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))

### Behavior Agent ([behavior_agent](/../paf/code/planning/src/behavior_agent/))

Decides which speed is the right one to pass through a certain situation and
also checks if an overtake is necessary.
Everything is based on the data from the Perception [Perception](#Perception). More about the behavior tree can be found [here](../planning/Behavior_tree.md)

Subscriptions:

[topics2blackboard.py](/../paf/code/planning/src/behavior_agent/behaviours/topics2blackboard.py)

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/slowed_by_car_in_front``` \(/\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/waypoint_distance``` \(/GlobalPlanDistance\) ([geographic_msgs/WayPoint](https://docs.ros.org/en/melodic/api/geographic_msgs/html/msg/WayPoint.html))
- ```/paf/hero/stop_sign``` \(/\) ([mock/Stop_sign](/../paf/code/mock/msg/Stop_sign.msg))
- ```/paf/hero/Center/traffic_light_state``` \(/TrafficLightNode\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))
- ```/paf/hero/Center/traffic_light_y_distance``` \(/TrafficLightNode\) ([std_msgs/Int16](https://docs.ros.org/en/api/std_msgs/html/msg/Int16.html))
- ```/paf/hero/max_velocity``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/speed_limit``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/lane_change_distance``` \(/GlobalPlanDistance\) ([perception/msg/LaneChange](/../paf/code/perception/msg/LaneChange.msg))
- ```/paf/hero/collision``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/overtake_success``` \(/MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/oncoming``` \(/CollisionCheck\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/target_velocity``` \(/MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

[maneuvers.py](/../paf/code/planning/src/behavior_agent/behaviours/maneuvers.py)

- ```/paf/hero/curr_behavior``` \(/MotionPlanning, /vehicle_controller\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/unstuck_distance``` \(/ACC, /MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/unstuck_flag``` \(/ACC\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

[intersection.py](/../paf/code/planning/src/behavior_agent/behaviours/intersection.py)

- ```/paf/hero/curr_behavior``` \(/MotionPlanning, /vehicle_controller\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
  
[lane_change.py](/../paf/code/planning/src/behavior_agent/behaviours/lane_change.py)

- ```/paf/hero/curr_behavior``` \(/MotionPlanning, /vehicle_controller\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

[meta.py](/../paf/code/planning/src/behavior_agent/behaviours/meta.py)

- ```/paf/hero/max_velocity``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

[overtake.py](/../paf/code/planning/src/behavior_agent/behaviours/overtake.py)

- ```/paf/hero/curr_behavior``` \(/MotionPlanning, /vehicle_controller\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

### [Local Planning](../planning/Local_Planning.md)

It consists of three components:

- [Collision Check](../planning//Collision_Check.md): Checks for collisions based on objects recieved from [Perception](#perception)
- [ACC](../planning/ACC.md): Generates a new speed based on a possible collision recieved from Collision Check and speedlimits recieved from [Global Planner](#global-planning)
- [Motion Planning](../planning/motion_planning.md): Decides the target speed and modifies trajectory if signal recieved from [Behavior Agent](#behavior-agent-behavior_agent)

Subscriptions:

[ACC.py](/../paf/code/planning/src/local_planner/ACC.py)

- ```/paf/hero/unstuck_flag``` \(/behavior_agent\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/unstuck_distance``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/speed_limits_OpenDrive``` \(/PrePlanner\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/trajectory_global``` \(/PrePlanner\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/collision``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))

[collision_check.py](/../paf/code/planning/src/local_planner/collision_check.py)

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/Center/object_distance``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))

[motion_planning.py](/../paf/code/planning/src/local_planner/motion_planning.py)

- ```/paf/hero/spawn_car``` \(/\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/speed_limit``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/trajectory_global``` \(/PrePlanner\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/curr_behavior``` \(/behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/unchecked_emergency``` \(/\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/acc_velocity``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/waypoint_distance``` \(/GlobalPlanDistance\) ([geographic_msgs/WayPoint](https://docs.ros.org/en/melodic/api/geographic_msgs/html/msg/WayPoint.html))
- ```/paf/hero/lane_change_distance``` \(/GlobalPlanDistance\) ([perception/msg/LaneChange](/../paf/code/perception/msg/LaneChange.msg))
- ```/paf/hero/collision``` \(/CollisionCheck\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/Center/traffic_light_y_distance``` \(/TrafficLightNode\) ([std_msgs/Int16](https://docs.ros.org/en/api/std_msgs/html/msg/Int16.html))
- ```/paf/hero/unstuck_distance``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_wp``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

Publishes:

[ACC.py](/../paf/code/planning/src/local_planner/ACC.py)

- ```/paf/hero/acc_velocity``` \(/MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_wp``` ([std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/speed_limit``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

[collision_check.py](/../paf/code/planning/src/local_planner/collision_check.py)

- ```/paf/hero/emergency``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/collision``` \(/ACC, /MotionPlanning\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/oncoming``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

[motion_planning.py](/../paf/code/planning/src/local_planner/motion_planning.py)

- ```/paf/hero/trajectory``` \(/pure_pursuit_controller\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/target_velocity``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/overtake_success``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

## Acting

The job of this component is to take the planned trajectory and target-velocities from the [Planning](#Planning) component and convert them into steering and throttle/brake controls for the CARLA-vehicle.

All information regarding research done about acting can be found [here](../research/acting/README.md).

Indepth information about the currently implemented acting Components can be found [here](../acting/README.md)!

### [MainFramePublisher](/../paf/code/acting/src/acting/MainFramePublisher.py)

Subscription:

- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

### [pure_pursuit_controller](/../paf/code/acting/src/acting/pure_pursuit_controller.py)

Calculates steering angles that keep the ego vehicle on the path given by
the [Local path planning](#Local-path-planning).

Subscription:

- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/paf/hero/pure_pursuit_steer``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/pure_p_debug``` \(/\) ([acting/msg/Debug](/../paf/code/acting/msg/Debug.msg))
  
### [stanley_controller](/../paf/code/acting/src/acting/stanley_controller.py)

Calculates steering angles that keep the ego vehicle on the path given by
the [Local path planning](#Local-path-planning).

Subscription:

- ```/paf/hero/trajectory_global``` \(/PrePlanner\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/paf/hero/stanley_steer``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/stanley_debug``` \(/\) ([acting/msg/StanleyDebug](/../paf/code/acting/msg/StanleyDebug.msg))

### [vehicle_controller](/../paf/code/acting/src/acting/vehicle_controller.py)

Subscription:

- ```/paf/hero/curr_behavior``` \(/behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/emergency``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/throttle``` \(/velocity_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/brake``` \(/velocity_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/reverse``` \(/velocity_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/pure_pursuit_steer``` \(/pure_pursuit_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/stanley_steer``` \(/pure_pursuit_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/carla/hero/vehicle_control_cmd``` \(/\) ([ros_carla_msgs/msg/CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaEgoVehicleControl.msg))
- ```/carla/hero/status``` \(/\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/controller``` \(/\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/emergency``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

### [velocity_controller](/../paf/code/acting/src/acting/velocity_controller.py)

Calculates acceleration values to drive the target-velocity given by the [Local path planning](#Local-path-planning).

For further indepth information about the currently implemented Vehicle Controller click [here](../acting/vehicle_controller.md)

Subscription:

- ```/paf/hero/target_velocity``` \(/MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))

Publishes:

- ```/paf/hero/throttle``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/brake``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/reverse``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
