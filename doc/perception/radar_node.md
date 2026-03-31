# Detailed Description of the Radar Node

## Table of Contents

- [1. General Functionality](#1-general-functionality)
- [2. Sensor Configuration](#2-sensor-configuration)
- [3. Initialization and ROS Parameters](#3-initialization-and-ros-parameters)
- [4. Data Processing](#4-data-processing)
- [5. ROS Topics](#5-ros-topics)
- [6. Conclusion](#6-conclusion)

## 1. General Functionality

The **Radar Node** processes data from multiple radar sensors to improve environmental perception. It enables the detection of objects, their speeds, and distances, contributing to applications such as overtaking safety, turning maneuvers, and adaptive cruise control.

Initially, two forward-facing radar sensors were used to enhance long-range visibility and clustering robustness.
However, after further evaluation, one of the sensors was repositioned to the rear to improve the detection of approaching vehicles, making overtaking and parking maneuvers safer. Additionally the horizontal FOV of the radar sensors was set to
130 degrees to provide a better perception of lateral objects on e.g. intersections.
This configuration balances forward and rearward perception while maintaining overall system reliability.

This decision aligns with the objectives shown in the provided image, highlighting the motivation for using radar sensors:

- **Additional Information**: Capturing object speeds and distant object data
- **Important Applications**: Enhancing overtaking safety, turning, and adaptive cruise control
- **Expansion**: Utilizing two radar sensors (front-left & rear-left) within CARLA simulation constraints.

## 2. Sensor Configuration

The sensors are configured as follows:

| Sensor | x    | y    | z   | Horizontal FOV | Vertical FOV | roll | pitch | yaw   |
| ------ | ---- | ---- | --- | -------------- | ------------ | ---- | ----- | ----- |
| RADAR0 | 2.0  | -1.5 | 0.5 | 130            | 0.1          | 0.0  | 0.0   | 0.0   |
| RADAR1 | -2.0 | -1.5 | 0.5 | 130            | 0.1          | 0.0  | 0.0   | 180.0 |

**Special Note:** RADAR1 is rotated 180° to face backward.

Without a rearfacing radar the car was not able to change lanes or exit parking spaces safely as it was not possible to detect oncoming traffic in time.
To optimize detection, we set the vertical field of view to 0.1. This eliminates unnecessary points that would otherwise be directed straight at the ground or far above obsacles. This configuration allows for full utilization of the 1500 points returned by each sensor per second.

The sensors operate at a rate of 20 Hz, resulting in 75 points per tick per sensor.

### 2.1 Decision for Radar Placement

We initially deployed two forward-facing radar sensors but later opted for a configuration with one front-facing and one rear-facing sensor. The decision was based on the following considerations:

| Configuration                               | ✅ Pros                                                                                                                                          | ❌ Cons                                                                                            |
| ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| **Two Forward-Facing Radars**               | Wider long-range field of view and Higher data density → More robust clustering                                                                 | Cannot directly detect speeds of approaching vehicles                                             |
| **One Front-Facing, One Rear-Facing Radar** | Detects speeds of approaching vehicles for safer overtaking & parking maneuvers and better detection of distant vehicles when making left turns | Slightly reduced clustering quality for front detection and narrower forward-facing field of view |

### 2.2 Alternative Radar Placement

In principle, it is possible to use four radar sensors instead of just two. However, due to the limitations in the CARLA qualifying phase, only two sensors are currently allowed.

A future improvement could involve dynamically activating two radars for qualifying and then switching to four radars once full autonomy is permitted. This approach would significantly enhance the perception system by covering blind spots and improving clustering reliability.

## 3 Initialization and ROS Parameters

At startup, several parameters are retrieved via `get_param` to configure the node’s behavior:

- `~dbscan_eps`: Maximum distance between two points in a cluster (default: `0.3`)
- `~dbscan_samples`: Minimum number of points per cluster (default: `3`)
- `~data_buffered`: Specifies whether sensor data should be buffered (default: `False`)
- `~data_buffer_time`: Time interval for processing buffered data (default: `0.1` seconds)
- `~enable_clustering`: Enables or disables clustering (default: `False`)
- `~enable_debug_info`: Enables additional debug information output (default: `False`)
- `~accelerometer_arrow_size`: Scaling factor for IMU arrow visualization (default: `2.0`)
- `~accelerometer_factor`: Scaling factor for pitch angle calculation (default: `0.05`)

## 4 Data Processing

- **Receiving Sensor Data:**
  - Sensor data is received from `/carla/hero/RADAR0` and `/carla/hero/RADAR1` as `PointCloud2` messages.
  - Data can either be buffered or processed immediately.
- **IMU Data Processing:**
  - The vehicle's pitch angle is calculated from IMU data to filter out ground reflections.
  - IMU data is processed in `imu_callback`, where the last five measurements of x- and z-acceleration are stored and averaged.
  - The pitch angle is computed using `atan2(accel_x_avg, accel_z_avg)` and adjusted using a scaling factor.
- **Time Control and Buffering:**
  - The `time_check` function ensures that processing occurs only at predefined time intervals.
  - The interval is configured with `self.data_buffer_time` (default: `0.1s`).
  - **Buffering Option:** If `~data_buffered` is enabled, sensor data is collected over a short period before processing. This allows for better clustering but is generally not used in the final implementation.
- **Data Filtering:**
  - Points below the calculated ground level are filtered out.
  - The `filter_points` function applies masking based on the computed pitch angle.
- **Clustering:**
Radar clustering is not the primary use case in the current system configuration.
By default, clustering is disabled and radar data is mainly used for velocity estimation and fusion with lidar-based entities in the mapping stage.
  - [`DBSCAN`](./README.md) is used for clustering radar points.
  - Clustering criteria: x, y, z and velocity.
  - **Clustering Parameters:**
    - `eps` (maximum distance between points in a cluster) = 0.3
    - `min_samples` (minimum number of points per cluster) = 3
- **Cluster velocity:**
  - The average velocity for each cluster is computed by first calculating of per-point velocity and then taking
    the mean of all points in that cluster.
    - to obtain correct absolute motion, radar velocity is compensated with ego-vehicle motion.
    - radar points are transformed back into the radar sensor coordinate system before motion compensation.
  - Disclaimer: Currently the angular velocity of the ego vehicle is not considered and might result in inacurate motion values while turning.
- **Additional debugging functions:**
  - These functions can be used to improve radar clustering without depending on the intermediate layer
    - filter_data: Filters data in x,y and z direction as well as maximum distance to the sensor.
    - create_pointcloud2: Creates a colored pointcloud of radar points.
    - generate_bounding_boxes, create_bounding_box_marker: Creates bounding boxes and markers of radar clusters.
    - generate_cluster_info: Generates a string with cluster information.
- **Output to intermediate layer:**
  - Radar points with ego-motion compensated velocity (`/paf/hero/Radar/compensated_points`)
  - These points contain spatial position and velocity information and are used for radar-lidar fusion in the mapping stage
  
  - Optional output (if clustering enabled):
    - points (clusterPointsNpArray): numpy array shape (N, 3)
    - point_indices (indexArray): numpy array with the shape (N)
    - object_speed_array (motionArray): numpy array with the shape (N)

### 4.1 Usage for Cross Traffic Detection

Radar data is also used to support cross-traffic detection at intersections.

In particular, the measured velocity information allows reliable detection of fast-approaching vehicles from the side. This improves decision-making in intersection scenarios, where dynamic objects are more relevant than purely static occupancy.

It should be noted that radar data is not used for standalone object detection (as clustering is deactivated by default), but rather to assign motion information to objects detected by other sensors (e.g., LiDAR or vision). Radar points that cannot be associated with existing objects are not considered.

The radar node itself does not directly perform the behavioral decision. Instead, it provides processed motion-related information that can be used in the planning and intersection logic.

### 4.2 Ego Motion Compensation and Velocity Estimation

Radar points are processed to compute ego-motion compensated velocities.

Each radar point contains Doppler velocity information. This velocity is interpreted along the radar line of sight and then compensated using the ego vehicle motion.

The compensation now includes two components:

- the translational ego speed from `/carla/hero/Speed`
- the yaw-rate-induced sensor motion derived from `/carla/hero/IMU`

The yaw-rate term is evaluated at the physical sensor offset of each radar. This reduces the false motion that stationary entities would otherwise receive while the ego vehicle rotates, for example when leaving the parking spot.

As a result, a motion vector is computed for each radar point. These per-point velocities are used for further processing in the mapping stage, especially for assigning radar-derived motion to lidar-based entities.

## 5. ROS Topics

| Topic                               | Type                               | Description                                      |
| ----------------------------------- | ---------------------------------- | ------------------------------------------------ |
| `/carla/hero/RADAR0`                | `sensor_msgs/PointCloud2`          | Input data from Radar 0                          |
| `/carla/hero/RADAR1`                | `sensor_msgs/PointCloud2`          | Input data from Radar 1                          |
| `/carla/hero/Speed`                 | `carla_msgs/CarlaSpeedometer`     | Input data from CarlaSpeedometer                 |
| `/paf/hero/Radar/Visualization`     | `sensor_msgs/PointCloud2`          | Visualization of clustered points                |
| `/paf/hero/Radar/Marker`            | `visualization_msgs/MarkerArray`   | Bounding boxes of clusters                       |
| `/paf/hero/Radar/clustered_points`  | `mapping.msg.ClusteredPointsArray` | Clustered radar points with velocity values      |
| `/paf/hero/Radar/ClusterInfo`       | `std_msgs/String`                  | JSON with cluster information                    |
| `/paf/hero/Radar/compensated_points`| `sensor_msgs/PointCloud2`          | Radar points with ego-motion compensated velocity|
| `/paf/hero/IMU`                     | `sensor_msgs/Imu`                  | Input data from the IMU sensor                   |

## 6. Additional Debugging Tool

For debugging raw radar measurements, the project also contains a dedicated tool:

- `perception/radar_raw_debugger.py`

This script can be used to inspect raw radar point cloud messages directly. It is useful for checking available fields, validating point cloud structure, and investigating sensor-specific issues during development.

## 7. Conclusion

This radar node enables robust processing of radar signals for object detection. By integrating IMU data, sensor data quality is improved. Radar-derived velocities are now used in the mapping stage to enrich lidar-based entities with motion information.

In addition to its original use cases, radar now also contributes to cross-traffic detection by providing velocity information for dynamic objects. This makes the perception pipeline more robust in intersection scenarios and improves the system’s reaction to fast-approaching vehicles.
