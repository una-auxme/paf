# Documentation: Lidar Distance Node

## 1. Integration into the Perception Pipeline

The `lidar_distance.py` node is part of the perception pipeline. This node processes LiDAR data to provide precise distance information. The extracted and processed data serves as a foundation for subsequent layers such as the Intermediate Layer and the Planning module.

## 2. Point Cloud Compensation

In the simulation environment (CARLA/Leaderboard), there is a mismatch between the lidar rotation frequency (10Hz) and the simulation frequency (20Hz), resulting in a 180 degree lidar data per simulation frame and a blindspot on missing data.

To achieve better spatial coverage, the node can buffer the current and previous frame. The core challenge is that the environment points from the previous scan must be compensated (transformed) to account for the ego vehicle's motion that occurred between the two frames.

### Modular Compensation

The node uses the Strategy pattern to allow switching between different compensation modes via configuration:

- `CompensationStrategy` (Abstract Base class) defines the interface for all modes.
- The Compensation object handles buffering, data preparation and returns the compensated point cloud.

The available compensation modes are:

### 2.1 NoCompensation (Baseline)

This strategy represents the simplest case, where no buffering, registration, or compensation is performed on the incoming LiDAR data.

The system simply passes the current point cloud ($P_{cur}$) as the final result, discarding all previous frame data.

$$
   P_{comp} = P_{cur}
$$

### 2.2 Buffer

This strategy performs basic point cloud accumulation across two consecutive frames without applying any geometric transformation or motion compensation.

The LiDAR data from the current frame ($P_{cur}$) and the previous frame ($P_{prev}$) are buffered and directly concatenated (joined).
Since no transformation is applied, the previous cloud remains in its original, outdated coordinate frame, leading to motion misaligned points if the ego vehicle moved between frames.

$$
   P_{comp} = P_{cur} \cup P_{prev}
$$

### 2.3 EgoMotionCompensation

This strategy uses external state information (Estimated Kalman Filter, EKF) to determine the vehicle's transformation between the current frame ($f_i$) and the previous frame ($f_{i-1}$).
Since LiDAR data is always relative to the sensor, we must apply a transformation that describes the positional delta between the vehicle's current and previous pose to correctly align the point clouds.

2.3.1 Point Cloud Separation

We first divide the previous point cloud, $P_{prev}$, into points belonging to the ego vehicle, $P_{ego}$, and points belonging to the static environment, $P_{env}$, such that:

$$P_{prev} = P_{ego} \cup P_{env}$$

2.3.2 Homogeneous Transformation Matrix ($T_i$)

The EKF provides both the translation components and rotation components, allowing us to define the homogeneous transformation matrix $T_i$ for a frame $f_i$ relative to the local position. The transformation matrix $T_i$ is defined as following:

$$T_{i} = \begin{pmatrix}
R_{i} & t_{i} \\
\mathbf{0}^T & 1
\end{pmatrix}$$

Components:

$R_i$ is the $3 \times 3$ rotation matrix representing the orientation (roll, pitch, yaw) of the ego vehicle:

$$R_{i} = \begin{pmatrix}
  r_{11} & r_{12} & r_{13} \\
  r_{21} & r_{22} & r_{23} \\
  r_{31} & r_{32} & r_{33}
  \end{pmatrix}$$

$t_{i}$ is the $3 \times 1$ translation vector, representing the vehicle's position $(x, y, z)$:

$$t_{i} = \begin{pmatrix}
  t_x \\
  t_y \\
  t_z
  \end{pmatrix}$$

$\mathbf{0}^T$ is a $1 \times 3$ row vector of zeros: $\begin{pmatrix} 0 & 0 & 0 \end{pmatrix}$.

2.3.3 Delta Transformation Matrix ($\Delta T$)

Having defined the homogenous transformation matrices for both $T_i$ (current pose) and $T_{i-1}$ (previous pose), we calculate the positional delta between the two frames.
This delta transformation matrix, $\Delta T$, transforms points from the current sensor frame ($f_i$) back to the coordinates of the previous frame ($f_{i-1}$):

$$
   \Delta T = T_{i-1} T_i^{-1}
$$

2.3.4 Point Cloud Compensation

The $\Delta T$ allows us to transform all environment points ($P_{env}$ from the previous frame), compensating for the ego-motion that occurred between $f_{i-1}$ and $f_i$. For any point $(x, y, z)^T \in P_{env}$, the compensated position $P'_{env}$ is calculated using homogeneous coordinates:

$$
   P'_{env} = \Delta T \begin{pmatrix} x \\ y \\ z \\ 1 \end{pmatrix}
$$

The final compensated point cloud, $P_{comp}$, is the union of the environment points from the previous frame ($P'_{env}$), the current frame's point cloud ($P_{cur}$), and the ego vehicle points ($P_{ego}$):

$$
   P_{comp} = P_{cur} \cup P'_{env} \cup P_{ego}
$$

### 2.4 LocalCompensation

This strategy uses the vehicle's local speed and heading change between the previous frame ($f_{i-1}$) and the current frame ($f_i$) to correct the static points.

2.4.1 Point Cloud Separation

First, the previous point cloud, $P_{prev}$, is split into two groups:

$$P_{prev} = P_{ego} \cup P_{env}$$

2.4.2 Motion Compensation Components

We determine how much the vehicle moved during the time interval $\Delta t = t_i - t_{i-1}$.

Translation Component ($d_x$)

The main movement is the displacement along the vehicle's forward (X) axis. This distance, $d_x$, is calculated from the average speed ($\bar{v}$) over $\Delta t$. This is the distance we need to undo on the static points.

$$d_x \approx \bar{v} \cdot \Delta t$$

Rotation Component ($R_{\Delta}$) (Optional)

This is the $3 \times 3$ rotation matrix used to correct for the change in heading ($\Delta \theta$). This correction is only applied if the heading change is significant enough to warrant it, which is controlled by a flag in the implementation. The rotation is around the Z-axis (yaw):

$$R_{\Delta} = \begin{pmatrix}
\cos(-\Delta \theta) & -\sin(-\Delta \theta) & 0 \\
\sin(-\Delta \theta) & \cos(-\Delta \theta) & 0 \\
0 & 0 & 1
\end{pmatrix}$$

Where $\Delta \theta = \theta_i - \theta_{i-1}$.

2.4.3 Compensation Procedure

The correction is applied to the static points ($P_{env}$) in two steps:

Translation (Required): The points are moved backward along the X-axis by subtracting the calculated distance $d_x$:

$$x_{comp} = x - d_x$$

Rotation (Optional): If the account_heading flag is set to True, the resulting coordinates are rotated using the $R_{\Delta}$ matrix to correct for the vehicle's turning.

The final compensated point cloud, $P_{comp}$, is the combined set of current points, corrected static points, and ego points:

$$P_{comp} = P_{cur} \cup P'_{env} \cup P_{ego}$$

## 3. Input: Received Data

### Incoming Topics

- **Topic Name:** `/carla/hero/LIDAR`
- **Data Type:** `sensor_msgs/PointCloud2`
- **Description:** Contains point cloud data from the LiDAR sensor, serving as raw input for distance measurement.

### Incoming Topics (Compensation Mode Dependent)

EgoMotionCompensation active:

- **Topic Name:** `/paf/hero/local_current_pos`
- **Data Type:** `geometry_msgs/PoseStamped`
- **Description:** Provides the pose (position and orientation) of the ego vehicle in the local coordinate frame.

LocalCompensation active:

- **Topic Name:** `/carla/hero/Speed`
- **Data Type:** `carla_msgs/CarlaSpeedometer`
- **Description:** Provides the linear speed of the vehicle.

- **Topic Name:** `/carla/hero/IMU`
- **Data Type:** `sensor_msgs/Imu`
- **Description:** Provides data used for calculating the heading.

## 4. Processing Pipeline

### Processing Steps

### 4.1 Reception of PointCloud2 Data

- ROS subscriber receives the LiDAR point cloud (`callback` method).

### 4.2 Motion Compensation of Point Cloud

- Compensates point cloud based on the compensation mode (NoCompensation, Buffer, EgoMotionCompensation, LocalCompensation)

### 4.3 Filtering and Preprocessing

- Removes points representing the ego vehicle (`start_clustering`).
- Filters out points below a certain height to avoid clustering the road surface (`start_clustering`).

### 4.4 Clustering the LiDAR Data

- Uses DBSCAN to group spatially related points (`start_clustering`).
- Removes noise points classified by DBSCAN (`cluster_labels != -1`).
- Generates bounding boxes for identified clusters (`generate_bounding_boxes`).
- **Publishes:**
  - Visualization markers → `self.marker_visualization_lidar_publisher.publish(marker_array)`
  - Clustered data → `self.clustered_points_publisher.publish(clustered_points_msg)`

### 4.5 Distance Image Calculation

- Computes distance images for various directions (`start_image_calculation`).
- Filters LiDAR points for specific viewpoints (`calculate_image`).
- Reconstructs distance images from LiDAR data (`reconstruct_img_from_lidar`).
- **Publishes:**
  - Distance images for different viewpoints → `self.publish_images(processed_images, data.header)`

## 5. Output: Published Topics

### Filtered Point Clouds

- **Topic Name:** `~point_cloud_topic` (Default: `/carla/hero/_filtered`)
- **Data Type:** `sensor_msgs/PointCloud2`
- **Description:** Contains filtered LiDAR data after noise suppression.

### Distance Images in Various Directions

- **Center:** `~image_distance_topic` (Default: `/paf/hero/Center/dist_array`)
- **Back:** `~image_distance_topic` (Default: `/paf/hero/Back/dist_array`)
- **Left:** `~image_distance_topic` (Default: `/paf/hero/Left/dist_array`)
- **Right:** `~image_distance_topic` (Default: `/paf/hero/Right/dist_array`)
- **Data Type:** `sensor_msgs/Image`
- **Description:** Contains the calculated minimum distance to objects in various directions. Although the _Back_, _Left_, and _Right_ directions are still actively processed in this node's image pipeline from PAF23, the current vision node only subscribes to and utilizes the _Center_S image.  
  Support for the other directions has been intentionally preserved to allow future teams to easily extend the system with additional camera perspectives if needed.

### Marker Visualization

- **Topic Name:** `~marker_topic` (Default: `/paf/hero/Lidar/Marker`)
- **Data Type:** `visualization_msgs/MarkerArray`
- **Description:** Displays the LiDAR point clouds as RViz markers for visualization.

### Clustered Points

- **Topic Name:** `~clustered_points_lidar_topic` (Default: `/paf/hero/Lidar/clustered_points`)
- **Data Type:** `ClusteredPointsArray`
- **Description:** Clusters LiDAR data for further downstream analysis in the intermediate layer.
