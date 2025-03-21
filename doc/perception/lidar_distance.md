# Documentation: Lidar Distance Node

## 1. Integration into the Perception Pipeline
The `lidar_distance.py` node is part of the perception pipeline. This node processes LiDAR data to provide precise distance information. The extracted and processed data serves as a foundation for subsequent layers such as the Intermediate Layer and the Planning module.

## 2. Input: Received Data
### Incoming Topics:
- **Topic Name:** `~source_topic` (Default: `/carla/hero/LIDAR`)
- **Data Type:** `sensor_msgs/PointCloud2`
- **Description:** Contains point cloud data from the LiDAR sensor, serving as raw input for distance measurement.

## 3. Processing Pipeline
### Processing Steps:
1. **Reception of PointCloud2 Data**
   - ROS subscriber receives the LiDAR point cloud (`callback` method).

2. **Filtering and Preprocessing**
   - Removes points representing the ego vehicle (`start_clustering`).
   - Filters out points below a certain height to avoid clustering the road surface (`start_clustering`).

3. **Clustering the LiDAR Data**
   - Uses DBSCAN to group spatially related points (`start_clustering`).
   - Removes noise points classified by DBSCAN (`cluster_labels != -1`).
   - Generates bounding boxes for identified clusters (`generate_bounding_boxes`).
   - **Publishes:**  
     - Visualization markers → `self.marker_visualization_lidar_publisher.publish(marker_array)`  
     - Clustered data → `self.clustered_points_publisher.publish(clustered_points_msg)`

4. **Distance Image Calculation**
   - Computes distance images for various directions (`start_image_calculation`).
   - Filters LiDAR points for specific viewpoints (`calculate_image`).
   - Reconstructs distance images from LiDAR data (`reconstruct_img_from_lidar`).
   - **Publishes:**  
     - Distance images for different viewpoints → `self.publish_images(processed_images, data.header)`

## 4. Output: Published Topics
### Filtered Point Clouds:
- **Topic Name:** `~point_cloud_topic` (Default: `/carla/hero/_filtered`)
- **Data Type:** `sensor_msgs/PointCloud2`
- **Description:** Contains filtered LiDAR data after noise suppression.

### Distance Images in Various Directions
- **Center:** `~image_distance_topic` (Default: `/paf/hero/Center/dist_array`)
- **Back:** `~image_distance_topic` (Default: `/paf/hero/Back/dist_array`)
- **Left:** `~image_distance_topic` (Default: `/paf/hero/Left/dist_array`)
- **Right:** `~image_distance_topic` (Default: `/paf/hero/Right/dist_array`)
- **Data Type:** `sensor_msgs/Image`
- **Description:** Contains the calculated minimum distance to objects in various directions. Although the *Back*, *Left*, and *Right* directions are still actively processed in this node's image pipeline from PAF23, the current vision node only subscribes to and utilizes the *Center* image. Support for the other directions has been intentionally preserved to allow future teams to easily extend the system with additional camera perspectives if needed.

### Marker Visualization:
- **Topic Name:** `~marker_topic` (Default: `/paf/hero/Lidar/Marker`)
- **Data Type:** `visualization_msgs/MarkerArray`
- **Description:** Displays the LiDAR point clouds as RViz markers for visualization.

### Clustered Points:
- **Topic Name:** `~clustered_points_lidar_topic` (Default: `/paf/hero/Lidar/clustered_points`)
- **Data Type:** `ClusteredPointsArray`
- **Description:** Clusters LiDAR data for further downstream analysis in the intermediate layer.

