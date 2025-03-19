# Detailed Description of the Radar Node

## 1. General Functionality
The **Radar Node** is responsible for processing radar data from multiple sensors to improve environmental perception. It enables the detection of objects, their speeds, and distances, contributing to applications such as overtaking safety, turning maneuvers, and adaptive cruise control. 

Initially, two forward-facing radar sensors were used to enhance long-range visibility and clustering robustness. However, after further evaluation, one of the sensors was repositioned to the rear to improve the detection of approaching vehicles, making overtaking and parking maneuvers safer. This configuration balances forward and rearward perception while maintaining overall system reliability.

This decision aligns with the objectives shown in the provided image, highlighting the motivation for using radar sensors:
- **Additional Information**: Capturing object speeds and distant object data
- **Important Applications**: Enhancing overtaking safety, turning, and adaptive cruise control
- **Expansion**: Utilizing two radar sensors (front-left & rear-left) within CARLA simulation constraints.

## 2. Sensor Configuration
The sensors are configured as follows:

| Sensor  | x   | y    | z   | Horizontal FOV | Vertical FOV |
|---------|-----|------|-----|----------------|--------------|
| RADAR0  | 2.0 | -1.5 | 0.5 | 25             | 0.1          |
| RADAR1  | -2.0 | -1.5 | 0.5 | 25             | 0.1          |

**Special Note:** RADAR1 is rotated 180° to face backward.

### 2.1 Decision for Radar Placement
We initially deployed two forward-facing radar sensors but later opted for a configuration with one front-facing and one rear-facing sensor. The decision was based on the following considerations:

| Configuration | Pros | Cons |
|--------------|------|------|
| **Two Forward-Facing Radars** | ✅ Wider long-range field of view <br> ✅ Higher data density → More robust clustering | ❌ Cannot directly detect speeds of approaching vehicles |
| **One Front-Facing, One Rear-Facing Radar** | ✅ Detects speeds of approaching vehicles for safer overtaking & parking maneuvers <br> ✅ Better detection of distant vehicles when making left turns | ❌ Slightly reduced clustering quality for front detection <br> ❌ Narrower forward-facing field of view |

2.2 Alternative Radar Placement

In principle, it is possible to use four radar sensors instead of just two. However, due to the limitations in the CARLA qualifying phase, only two sensors are currently allowed.

A future improvement could involve dynamically activating two radars for qualifying and then switching to four radars once full autonomy is permitted. This approach would significantly enhance the perception system by covering blind spots and improving clustering reliability.

## 3. Main Components

### 3.1 Initialization and ROS Parameters
At startup, several parameters are retrieved via `get_param` to configure the node’s behavior:

- `~dbscan_eps`: Maximum distance between two points in a cluster (default: `0.3`)
- `~dbscan_samples`: Minimum number of points per cluster (default: `3`)
- `~data_buffered`: Specifies whether sensor data should be buffered (default: `False`)
- `~data_buffer_time`: Time interval for processing buffered data (default: `0.1` seconds)
- `~enable_clustering`: Enables or disables clustering (default: `False`)
- `~enable_debug_info`: Enables additional debug information output (default: `False`)
- `~accelerometer_arrow_size`: Scaling factor for IMU arrow visualization (default: `2.0`)
- `~accelerometer_factor`: Scaling factor for pitch angle calculation (default: `0.05`)

### 3.2 Data Processing
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

### 3.3 Data Filtering
- Points below the calculated ground level are filtered out.
- Points outside the maximum detection range (100m) are removed.
- The `filter_points` function applies masking based on the computed pitch angle.

### 3.4 Clustering
- DBSCAN is used for grouping radar points.
- **Clustering Parameters:**
  - `eps` (maximum distance between points in a cluster) = 0.3
  - `min_samples` (minimum number of points per cluster) = 3
- The average velocity per cluster is calculated.
- Clustering can be disabled via the `~enable_clustering` parameter.

### 3.5 Data Output
- **Filtered Points:** Stored in `filtered_out_points` and published for visualization.
- **Clustered Points:** Published as `PointCloud2` messages.
- **Bounding Boxes:** An axis-aligned bounding box (AABB) is computed for each cluster and displayed in RViz.
- **Cluster Information:** Metadata on clusters is provided in JSON format.

## 4. ROS Topics
| Topic | Type | Description |
|----------------------------|---------------------------|--------------------------------------------------|
| `/carla/hero/RADAR0` | `sensor_msgs/PointCloud2` | Input data from Radar 0 |
| `/carla/hero/RADAR1` | `sensor_msgs/PointCloud2` | Input data from Radar 1 |
| `/paf/hero/Radar/Visualization` | `sensor_msgs/PointCloud2` | Visualization of clustered points |
| `/paf/hero/Radar/Marker` | `visualization_msgs/MarkerArray` | Bounding boxes of clusters |
| `/paf/hero/Radar/clustered_points` | `mapping.msg.ClusteredPointsArray` | Clustered radar points with velocity values |
| `/paf/hero/Radar/ClusterInfo` | `std_msgs/String` | JSON with cluster information |
| `/paf/hero/IMU` | `sensor_msgs/Imu` | Input data from the IMU sensor |

## 5. Conclusion
This radar node enables robust processing of radar signals for object detection. By integrating DBSCAN clustering and IMU data, sensor data quality is improved. The generated bounding boxes and visualizations facilitate environmental analysis.

