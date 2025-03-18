# Current architecture of the vehicle agent

**Summary:** This page gives an overview over the current general architecture of the vehicle agent.
The document contains an overview over all [nodes](#overview) and [topics](#topics). Additionally a visual thematic grouping of the subsystems of the agent is done for an easier optical identification which nodes and topics belong/contribute to them.

- [Overview](#overview)
- [Perception](#perception)
  - [Vision Node (vision\_node.py)](#vision-node-vision_nodepy)
  - [Traffic Light Detection (traffic\_light\_node.py)](#traffic-light-detection-traffic_light_nodepy)
  - [Lidar Node (lidar\_distance.py)](#lidar-node-lidar_distancepy)
  - [Lanedetection (Lanedetection\_node.py)](#lanedetection-lanedetection_nodepy)
  - [Radar Node (radar\_node.py)](#radar-node-radar_nodepy)
  - [Laneposition (lane\_position.py)](#laneposition-lane_positionpy)
- [Localization](#localization)
  - [Ekf state publisher (ekf\_state\_publisher.py)](#ekf-state-publisher-ekf_state_publisherpy)
  - [Position Heading Publisher Node (position\_heading\_publisher\_node.py)](#position-heading-publisher-node-position_heading_publisher_nodepy)
- [Mapping](#mapping)
  - [Mapping Data Integration (mapping\_data\_integration.py)](#mapping-data-integration-mapping_data_integrationpy)
  - [Mapping Visualization (visualization.py)](#mapping-visualization-visualizationpy)
- [Planning](#planning)
  - [ACC (ACC.py)](#acc-accpy)
  - [MotionPlanning (motion\_planning.py)](#motionplanning-motion_planningpy)
  - [PrePlanner (global\_planner\_node.py)](#preplanner-global_planner_nodepy)
  - [GlobalPlanDistance (global\_plan\_distance\_publisher.py)](#globalplandistance-global_plan_distance_publisherpy)
  - [Behavior Agent (behavior\_agent)](#behavior-agent-behavior_agent)
- [Acting](#acting)
  - [Passthrough (passthrough.py)](#passthrough-passthroughpy)
  - [velocity\_controller](#velocity_controller)
  - [pure\_pursuit\_controller](#pure_pursuit_controller)
  - [vehicle\_controller](#vehicle_controller)

## Overview

The vehicle agent is split into five major components: [Perception](#Perception), [Localization](#Localization), [Mapping](#Mapping), [Planning](#Planning)
and [Acting](#Acting).
The topics published by the Carla bridge can be
found [here](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/).\
The messages necessary to control the vehicle via the Carla bridge can be
found [here](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg).

The drawio-file can be found [here](../assets/nodes_visualization.drawio).

![Architecture overview](../assets/nodes_visualization.svg)
*Connections between nodes visualized*

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
More information under [vision_node.md](/doc/perception/vision_node.md).

Subscriptions:

- ```/paf/hero/Center/dist_array``` \(/lidar_distance\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Center/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/VisionNode/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/VisionNode/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))
- ```/paf/hero/Center/segmented_image``` \(/rviz\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Center/segmented_traffic_light``` \(/TrafficLightNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/visualization_pointcloud``` ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))

Services:

- ```/VisionNode/get_loggers```
- ```/VisionNode/set_logger_level```
- ```/VisionNode/set_parameters```

### Traffic Light Detection ([traffic_light_node.py](/../paf/code/perception/src/traffic_light_node.py))

Recognizes traffic lights and what they are showing at the moment.
In particular traffic lights that are relevant for the correct traffic behavior of the ego vehicle,
are recognized early and reliably.
More information under [traffic_light_detection.md](/doc/perception/traffic_light_detection.md).

Subscriptions:

- ```/paf/hero/Center/segmented_traffic_light``` \(/VisionNode\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/paf/hero/Center/traffic_light_state``` \(/behavior_agent\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))
- ```/paf/hero/TrafficLight/state/debug_marker``` \(/behavior_agent\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))

Services:

- ```/TrafficLightNode/get_loggers```
- ```/TrafficLightNode/set_logger_level```

### Lidar Node ([lidar_distance.py](/../paf/code/perception/src/lidar_distance.py))

Processes LIDAR point clouds by filtering, clustering, generating bounding boxes and publishing the results.

Subscriptions:

- ```/carla/hero/LIDAR``` \(/rosbridge_websocket\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))

Publishes:

- ```/carla/hero/LIDAR_filtered``` \(no subscriber at the moment\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- ```/paf/hero/Back/dist_array``` \(no subscriber at the moment\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Center/dist_array``` \(lane_position\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Left/dist_array``` \(no subscriber at the moment\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Lidar/Marker``` \(no subscriber at the moment\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- ```/paf/hero/Lidar/clustered_points``` \(mapping_data_integration\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))
- ```/paf/hero/Right/dist_array``` \(no subscriber at the moment\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Services:

- ```/lidar_distance/get_loggers```
- ```/lidar_distance/set_logger_level```

### Lanedetection ([Lanedetection_node.py](/../paf/code/perception/src/Lanedetection_node.py))

More information under [Lanedetection_node.md](/doc/perception/Lanedetection_node.md).

Subscriptions:

- ```/carla/hero/Center/image``` \(/carla_ros_bridge\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Center/dist_array``` \(lidar_distance\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/carla/hero/Center/driveable_area``` \(no subscriber at the moment\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/carla/hero/Center/lane_mask``` \(/lane_position\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Services:

- ```/Lanedetection/get_loggers```
- ```/Lanedetection/set_logger_level```

### Radar Node ([radar_node.py](/../paf/code/perception/src/radar_node.py))

More information under [radar_node.md](/doc/perception/radar_node.md).

Processes radar point clouds by filtering, clustering, generating bounding boxes and publishing the results.

Subscriptions:

- ```/carla/hero/RADAR0``` \(/rosbridge_websocket\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- ```/carla/hero/RADAR1``` \(/rosbridge_websocket\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- ```/carla/hero/IMU``` \(/rosbridge_websocket\) ([sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))

Publishes:

- ```/paf/hero/Radar/ClusterInfo``` \(no subscriber at the moment\) ([std_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/paf/hero/Radar/IMU/ground_filter/debug_marker``` \(no subscriber at the moment\) ([visualization_msgs/Marker](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
- ```/paf/hero/Radar/Marker``` \(no subscriber at the moment\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- ```/paf/hero/Radar/clustered_points``` \(mapping_data_integration\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))
- ```/paf/hero/Radar/combined_points``` \(no subscriber at the moment\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))
- ```/carla/hero/radar_break_filtered``` \(no subscriber at the moment\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))

Services:

- ```/radar_node/get_loggers```
- ```/radar_node/set_logger_level```

### Laneposition ([lane_position.py](/../paf/code/perception/src/lane_position.py))

More information under [lane_position.md](/doc/perception/lane_position.md).

Subscriptions:

- ```/carla/hero/Center/lane_mask``` \(/Lanedetection\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/Center/dist_array``` \(lidar_distance\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

Publishes:

- ```/paf/hero/Lane/label_image``` \(/position_heading_publisher_node\) ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
- ```/paf/hero/mapping/init_lanemarkings``` \(/mapping_data_integration\) ([mapping/Map](../../code/mapping/msg/Map.msg))

Services:

- ```/lane_position/get_loggers```
- ```/lane_position/set_logger_level```

## Localization

For the localization the robot_localization package is used.

### Ekf state publisher ([ekf_state_publisher.py](/../paf/code/localization/src/ekf_state_publisher.py))

Subscriptions:

- ```/tf``` \(/tf\) ([tf2_msgs/TFMessage](https://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html))
- ```/tf_static``` \(tf\) [unknown type]

Publishes:

- ```/paf/hero/ekf_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/ekf_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

Services:

- ```/ekf_state_publisher/get_loggers```
- ```/ekf_state_publisher/set_logger_level```
- ```/ekf_state_publisher/tf2_frames```

### Position Heading Publisher Node ([position_heading_publisher_node.py](/../paf/code/localization/src/position_heading_publisher_node.py))

More information can be found [here](/doc/localization/architecture.md).

Subscriptions:

- ```/carla/hero/GPS``` \(/carla_ros_bridge\) ([sensor_msgs/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))
- ```/carla/hero/IMU``` \(/carla_ros_bridge\) ([sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))
- ```/carla/hero/OpenDrive``` \(/carla_ros_bridge\) ([std_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/paf/hero/ekf_heading``` \(/ekf_state_publisher\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/ekf_pos``` \(/ekf_state_publisher\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

Publishes:

- ```/paf/hero/current_heading``` \(/MotionPlanning, /mapping_data_integration, /behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_pos``` \(/PrePlanner, /MotionPlanning, /mapping_data_integration, /GlobalPlanDistance, /behavior_agent\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/hero/unfiltered_heading``` \(/no subscriber at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/unfiltered_pos``` \(/no subscriber at the moment\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

Services:

- ```/position_heading_publisher_node/get_loggers```
- ```/position_heading_publisher_node/set_logger_level```

## Mapping

The so called intermediate layer.

### Mapping Data Integration ([mapping_data_integration.py](/../paf/code/mapping/src/mapping_data_integration.py))

More information can be found [here](/code/mapping/README.md).

Subscriptions:

- ```/carla/hero/LIDAR``` \(/rosbridge_websocket\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/Lidar/clustered_points``` \(/lidar_distance\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))
- ```/paf/hero/Radar/clustered_points``` \(/radar_node\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/hero/mapping/init_lanemarkings``` \(/lane_position\) ([mapping/Map](../../code/mapping/msg/Map.msg))
- ```/paf/hero/visualization_pointcloud``` \(/VisionNode\) ([mapping/ClusturedPointsArray](../../code/mapping/msg/ClusteredPointsArray.msg))

Publishes:

- ```/mapping_data_integration/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/mapping_data_integration/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))
- ```/paf/hero/mapping/clusterpoints``` \(no subscriber at the moment\) ([sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- ```/paf/hero/mapping/init_data``` \(/mapping_visualization, /ACC, /behavior_agent\) ([mapping/Map](../../code/mapping/msg/Map.msg))

Services:

- ```/mapping_data_integration/get_loggers```
- ```/mapping_data_integration/set_logger_level```
- ```/mapping_data_integration/set_parameters```
- ```/paf/hero/mapping/update_stop_marks```

### Mapping Visualization ([visualization.py](/../paf/code/mapping_visualization/src/visualization.py))

Subscriptions:

- ```/paf/hero/mapping/init_mapping``` \(/mapping_data_integration\) ([mapping/Map](../../code/mapping/msg/Map.msg))

Publishes:

- ```/paf/hero/mapping/marker_array``` \(/rviz\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))

Services:

- ```/mapping_visualization/get_loggers```
- ```/mapping_visualization/set_logger_level```
- ```/mapping_visualization/set_parameters```

## Planning

The planning uses the data from the [Perception](#Perception) to find a path on which the ego vehicle can safely reach its destination. It also detects situations and reacts accordingly in traffic. It publishes signals such as a trajecotry or a target speed to acting.

Further information regarding the planning can be found [here](../planning/README.md).
Research for the planning can be found [here](../research/planning/README.md).

### ACC ([ACC.py](/../paf/code/planning/src/local_planner/ACC.py))

More information under [ACC.md](/doc/planning/ACC.md).

Subscriptions:

- ```/paf/hero/curr_behavior``` \(/behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/mapping/init_mapping``` \(/mapping_data_integration\) ([mapping/Map](../../code/mapping/msg/Map.msg))
- ```/paf/hero/pure_pursuit_steer``` \(/pure_pursuit_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/speed_limit``` \(/MotionPlanning\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/trajectory_local``` \(/MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/ACC/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/ACC/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))
- ```/paf/hero/acc/debug_markers``` \(/rviz\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))
- ```/paf/hero/acc_velocity``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/emergency``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

Services:

- ```/ACC/get_loggers```
- ```/ACC/set_logger_level```
- ```/ACC/set_parameters```
- ```/paf/hero/acc/speed_alteration```

### MotionPlanning ([motion_planning.py](/../paf/code/planning/src/local_planner/motion_planning.py))

Uses information from the map and the path specified by CARLA to find a first concrete path to the next intermediate point. More about it can be found [here](../planning/motion_planning.md).

Subscriptions:

- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/hero/speed_limits_OpenDrive``` \(/PrePlanner\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/trajectory_global``` \(/PrePlanner\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/paf/hero/speed_limit``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/trajectory``` \(no subscribers at the moment\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/trajectory_local``` \(/ACC, /rviz, /GlobalPlanDistance, /behavior_agent\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Services:

- ```/MotionPlanning/get_loggers```
- ```/MotionPlanning/set_logger_level```
- ```/paf/hero/motion_planning/end_overtake```
- ```/paf/hero/motion_planning/overtake_status```
- ```/paf/hero/motion_planning/start_overtake```

### PrePlanner ([global_planner_node.py](/../paf/code/planning/src/global_planner/global_planner_node.py))

Uses information from the map and the path specified by CARLA to find a first concrete path to the next intermediate point. More about it can be found [here](../planning/Global_Planner.md).

Subscriptions:

- ```/carla/hero/OpenDrive``` \(/carla_ros_bridge\) ([std_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- ```/carla/hero/global_plan``` \(/ros_websocket\) ([carla_msgs/CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

Publishes:

- ```/paf/hero/speed_limits_OpenDrive``` \(/MotionPlanning\) ([sensor_msgs/Float32MultiArray](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- ```/paf/hero/trajectory_global``` \(/MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Services:

- ```/PrePlanner/get_loggers```
- ```/PrePlanner/set_logger_level```

### GlobalPlanDistance ([global_plan_distance_publisher.py](/../paf/code/planning/src/global_planner/global_plan_distance_publisher.py))

Subscriptions:

- ```/carla/hero/global_plan``` \(/ros_websocket\) ([carla_msgs/CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/hero/trajectory_local``` \(/MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/paf/hero/current_waypoint``` \(/behavior_agent\) ([perception/Waypoint](/code/perception/msg/Waypoint.msg))

Services:

- ```/GlobalPlanDistance/get_loggers```
- ```/GlobalPlanDistance/set_logger_level```

### Behavior Agent ([behavior_agent](/../paf/code/planning/src/behavior_agent/))

Decides which speed is the right one to pass through a certain situation and
also checks if an overtake is necessary.
Everything is based on the data from the Perception [Perception](#Perception). More about the behavior tree can be found [here](../planning/Behavior_tree.md)

Subscriptions:

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/msg/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/Center/traffic_light_state``` \(/TrafficLightNode\) ([perception/TrafficLightState](/../paf/code/perception/msg/TrafficLightState.msg))
- ```/paf/hero/acc_velocity``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/curr_behavior``` \(/behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/current_waypoint``` \(/GlobalPlanDistance\) ([geographic_msgs/WayPoint](https://docs.ros.org/en/melodic/api/geographic_msgs/html/msg/WayPoint.html))
- ```/paf/hero/mapping/init_data``` \(/mapping_data_integration\) ([mapping/Map](../../code/mapping/msg/Map.msg))
- ```/paf/hero/target_velocity``` \(/no subscriptions at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/trajectory_local``` \(/MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/behavior_agent/ascii/snapshot``` \(/no subscribers at the moment\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/behavior_agent/ascii/tree``` \(/no subscribers at the moment\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/behavior_agent/blackboard``` \(/no subscribers at the moment\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/behavior_agent/dot/tree``` \(/no subscribers at the moment\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/behavior_agent/introspection/publishers``` \(/no subscribers at the moment\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/behavior_agent/log/tree``` \(/no subscribers at the moment\) ([py_trees_msgs/BehaviourTree](https://docs.ros.org/en/kinetic/api/py_trees_msgs/html/msg/Behaviour.html))
- ```/behavior_agent/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/behavior_agent/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))
- ```/behavior_agent/tip``` \(/no subscribers at the moment\) ([py_trees_msgs/Behaviour](https://docs.ros.org/en/kinetic/api/py_trees_msgs/html/msg/Behaviour.html))
- ```/paf/hero/behavior_agent/debug_markers``` \(/rviz\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- ```/paf/hero/behavior_agent/info_marker``` \(/rviz\) ([visualization_msgs/Marker](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
- ```/paf/hero/curr_behavior``` \(/ACC, /behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/overtake/debug_markers``` \(/rviz\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- ```/paf/hero/overtake_distance``` \(no subscriber at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Services:

- ```/behavior_agent/close_blackboard_watcher```
- ```/behavior_agent/get_blackboard_variables```
- ```/behavior_agent/get_loggers```
- ```/behavior_agent/open_blackboard_watcher```
- ```/behavior_agent/set_logger_level```
- ```/behavior_agent/set_parameters```

## Acting

The job of this component is to take the planned trajectory and target-velocities from the [Planning](#Planning) component and convert them into steering and throttle/brake controls for the CARLA-vehicle.

All information regarding research done about acting can be found [here](../research/acting/README.md).

Indepth information about the currently implemented acting Components can be found [here](../acting/README.md)!

### Passthrough ([passthrough.py](/../paf/code/acting/src/acting/passthrough.py))

More information under [passthrouhg.md](/doc/acting/passthrough.md).

Subscriptions:

- ```/paf/hero/acc_velocity``` \(/ACC\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/hero/trajectory_local``` \(/MotionPlanning\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/paf/acting/current_heading``` \(no subscriber at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/acting/current_pos``` \(no subscriber at the moment\) ([geometry_msgs/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
- ```/paf/acting/target_velocity``` \(/velocity_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/acting/trajectory_local``` \(/pure_pursuit_controller\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/target_velocity``` \(/behavior_agent\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Services:

- ```/passthrough/get_loggers```
- ```/passthrough/set_logger_level```

### [velocity_controller](/../paf/code/control/src/velocity_controller.py)

Calculates acceleration values to drive the target-velocity given by the [Local path planning](#Local-path-planning).

For further indepth information about the currently implemented Vehicle Controller click [here](../control/vehicle_controller.md)

Subscription:

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/acting/target_velocity``` \(/passthrough\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/paf/hero/brake``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/reverse``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/throttle``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/velocity_controller/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/velocity_controller/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))

Services:

- ```/velocity_controller/get_loggers```
- ```/velocity_controller/set_logger_level```
- ```/velocity_controller/set_parameters```

### [pure_pursuit_controller](/../paf/code/control/src/pure_pursuit_controller.py)

Calculates steering angles that keep the ego vehicle on the path given by
the [Local path planning](#Local-path-planning).

Subscription:

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/acting/trajectory_local``` \(/passthrough\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- ```/paf/hero/control/pp_debug_markers``` \(/rviz\) ([visualization_msgs/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- ```/paf/hero/pure_p_debug``` \(no subscriber at the moment\) ([acting/Debug](/code/acting/msg/Debug.msg))
- ```/paf/hero/pure_pursuit_steer``` \(/ACC, /vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/pure_pursuit_controller/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/pure_pursuit_controller/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))

Services:

- ```/pure_pursuit_controller/get_loggers```
- ```/pure_pursuit_controller/set_logger_level```
- ```/pure_pursuit_controller/set_parameters```

### [vehicle_controller](/../paf/code/control/src/vehicle_controller.py)

More information under [vehicle_controller.md](/doc/control/vehicle_controller.md).

Subscription:

- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([ros_carla_msgs/CarlaSpeedometer](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaSpeedometer.msg))
- ```/paf/hero/brake``` \(/velocity_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/curr_behavior``` \(/behavior_agent\) ([std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
- ```/paf/hero/emergency``` \(/ACC, /vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/pure_pursuit_steer``` \(/pure_pursuit_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/reverse``` \(/velocity_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/paf/hero/throttle``` \(/velocity_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/carla/hero/status``` \(no subscriber at the moment\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/carla/hero/vehicle_control_cmd``` \(/odometry_fusion, /rosbridge_websocket\) ([ros_carla_msgs/msg/CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaEgoVehicleControl.msg))
- ```/paf/hero/controller``` \(no subscriber at the moment\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/emergency``` \(/vehicle_controller\) ([std_msg/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- ```/vehicle_controller/parameter_description```  ([dynamic_reconfigure/ConfigDescription](https://wiki.ros.org/dynamic_reconfigure))
- ```/vehicle_controller/parameter_update``` ([dynamic_reconfigure/Config](https://wiki.ros.org/dynamic_reconfigure))

Services:

- ```/vehicle_controller/get_loggers```
- ```/vehicle_controller/set_logger_level```
- ```/vehicle_controller/set_parameters```
