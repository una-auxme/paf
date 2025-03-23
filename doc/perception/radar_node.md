# Detailed Description of the Radar Node

## 1. General Functionality

The **Radar Node** processes data from multiple radar sensors to improve environmental perception. It enables the detection of objects, their speeds, and distances, contributing to applications such as overtaking safety, turning maneuvers, and adaptive cruise control. 

Initially, two forward-facing radar sensors were used to enhance long-range visibility and clustering robustness. However, after further evaluation, one of the sensors was repositioned to the rear to improve the detection of approaching vehicles, making overtaking and parking maneuvers safer. This configuration balances forward and rearward perception while maintaining overall system reliability.

This decision aligns with the objectives shown in the provided image, highlighting the motivation for using radar sensors:

- **Additional Information**: Capturing object speeds and distant object data
- **Important Applications**: Enhancing overtaking safety, turning, and adaptive cruise control
- **Expansion**: Utilizing two radar sensors (front-left & rear-left) within CARLA simulation constraints.

## 2. Sensor Configuration

The sensors are configured as follows:

| Sensor | x    | y    | z   | Horizontal FOV | Vertical FOV | roll | pitch | yaw   |
| ------ | ---- | ---- | --- | -------------- | ------------ | ---- | ----- | ----- |
| RADAR0 | 2.0  | -1.5 | 0.5 | 25             | 0.1          | 0.0  | 0.0   | 0.0   |
| RADAR1 | -2.0 | -1.5 | 0.5 | 25             | 0.1          | 0.0  | 0.0   | 180.0 |

**Special Note:** RADAR1 is rotated 180° to face backward.

Without a rearfacing radar the car was not able to change lanes or exit parking spaces safely as it was not possible to detect oncoming traffic in time. 

To optimize detection, we set the vertical field of view to 0.1. This eliminates unnecessary points that would otherwise be directed straight at the ground or far above obsacles. This configuration allows for full utilization of the 1500 points returned by each sensor per second.

The sensors operate at a rate of 20 Hz, resulting in 75 points per tick per sensor.

### 2.1 Decision for Radar Placement

We initially deployed two forward-facing radar sensors but later opted for a configuration with one front-facing and one rear-facing sensor. The decision was based on the following considerations:

| Configuration                               | Pros                                                                                                                                                 | Cons                                                                                                   |
| ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| **Two Forward-Facing Radars**               | ✅ Wider long-range field of view <br> ✅ Higher data density → More robust clustering                                                                 | ❌ Cannot directly detect speeds of approaching vehicles                                                |
| **One Front-Facing, One Rear-Facing Radar** | ✅ Detects speeds of approaching vehicles for safer overtaking & parking maneuvers <br> ✅ Better detection of distant vehicles when making left turns | ❌ Slightly reduced clustering quality for front detection <br> ❌ Narrower forward-facing field of view |

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
  - [`DBSCAN`](./README.md) is used for clustering radar points.
  - Clustering criteria: x, y, z and velocity.
  - **Clustering Parameters:**
    - `eps` (maximum distance between points in a cluster) = 0.3
    - `min_samples` (minimum number of points per cluster) = 3
- **Cluster velocity:**
  - The average velocity for each cluster is calculated.
- **Additional debugging functions:**
  - These functions can be used to improve radar clustering without depending on the intermediate layer
    - filter_data: Filters data in x,y and z direction as well as maximum distance to the sensor.
    - create_pointcloud2: Creates a colored pointcloud of radar points.
    - generate_bounding_boxes, create_bounding_box_marker: Creates bounding boxes and markers of radar clusters.
    - generate_cluster_info: Generates a string with cluster information.
- **Output to intermediate layer:**
  - Clustered points array containing:
    - points (clusterPointsNpArray): numpy array shape (N, 3)
    - point_indices (indexArray): numpy array with the shape (N)
    - object_speed_array (motionArray): numpy array with the shape (N) 

## 5. ROS Topics

| Topic                              | Type                               | Description                                 |
| ---------------------------------- | ---------------------------------- | ------------------------------------------- |
| `/carla/hero/RADAR0`               | `sensor_msgs/PointCloud2`          | Input data from Radar 0                     |
| `/carla/hero/RADAR1`               | `sensor_msgs/PointCloud2`          | Input data from Radar 1                     |
| `/paf/hero/Radar/Visualization`    | `sensor_msgs/PointCloud2`          | Visualization of clustered points           |
| `/paf/hero/Radar/Marker`           | `visualization_msgs/MarkerArray`   | Bounding boxes of clusters                  |
| `/paf/hero/Radar/clustered_points` | `mapping.msg.ClusteredPointsArray` | Clustered radar points with velocity values |
| `/paf/hero/Radar/ClusterInfo`      | `std_msgs/String`                  | JSON with cluster information               |
| `/paf/hero/IMU`                    | `sensor_msgs/Imu`                  | Input data from the IMU sensor              |

## 6. Conclusion

This radar node enables robust processing of radar signals for object detection. By integrating DBSCAN clustering and IMU data, sensor data quality is improved. The generated bounding boxes and visualizations facilitate environmental analysis.

