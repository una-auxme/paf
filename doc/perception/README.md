# Documentation of Perception

This folder contains documentation of the perception components.

In the following diagram you can see all the nodes and topics used in the perception, to show how they work together.

![Perception Overview](../../doc/assets/perception/perception_overview.png)


## Sensor overview 

![Perception Overview](../../doc/assets/perception/carla_sensor_overview.png)

The CARLA simulation vehicle is equipped with multiple sensors to perceive its surroundings and gather essential driving data:

- **GNSS, IMU, Speedometer, OpenDrive Map (Center of the Car - Red Dot):**
  - Position: x=0, y=0, z=0 
  - **GNSS (GPS):** Provides geolocation data for vehicle positioning.  
  - **IMU:** Measures acceleration and rotational speed to provide motion data.  
  - **Speedometer:** Estimates the vehicle's linear velocity.  
  - **OpenDrive Map:** Supplies HD map data in OpenDRIVE format, parsed as a string.  

- **Radar (Dark Blue Blocks on the Sides):**
  - Position: x=2/-2, y=1.5, z=0.5
  - Fov: vertical = 0.1°, horicontal = 25°
  - Detects objects and measures their relative velocity using radio waves up to 100m.  

- **Lidar (Purple Circle at the Center):**
  - Position: x=0, y=0, z=1.7 
  - Uses laser pulses to generate a high-resolution 3D representation of the surroundings.  

- **RGB Camera (Green Block at the Front):**
  - Position: x=0, y=0, z=1.7
  - Fov: 100°
  - Captures visual information to support object detection and scene analysis.  

([Source: leaderboard.carla.org](https://leaderboard.carla.org/get_started_v2_0/?utm_source=chatgpt.com))

## Object Detection / Distance and Segmentation

- [Vision Node](./vision_node.md)
  - The Vision Node provides an adaptive interface that is able to perform object detection and image segmentation on multiple cameras at the same time
  (even though only one camera perceiving the front of the car is active at the moment).
- [Distance to Objects](./distance_to_objects.md)
- [Traffic Light Detection](./traffic_light_detection.md)
- [Dataset Generator](./dataset_generator.md)
- [Dataset Structure](./dataset_structure.md)

## Localization

An overview over the different nodes working together to localize the vehicle is provided in the [localization](./localization.md) file.

- [Kalman Filter](./kalman_filter.md)
- [Position Heading Publisher Node](./position_heading_publisher_node.md)
- [Position Heading Filter Debug Node](./position_heading_filter_debug_node.md)
- [Coordinate Transformation](./coordinate_transformation.md) (helper functions)

## Unused files

- [Lidar Distance Utility](./lidar_distance_utility.md)
  - Not used since paf22
- [Efficient PS](./efficientps.md)
  - Not used scince paf22 and never successfully tested

## Experiments

The overview of performance evaluations is located in the [experiments](./experiments/README.md) folder.
