# Lidar Distance Utility

!!! No longer used after **paf22** !!!

**Summary:**

This node can filter points from a [PoinCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
which are located in a given bounding box.
It then publishes a filtered [PoinCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) containing
the points located in the given restriction.
Additionally, it publishes a [Range](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html) message
containing the closest and the farest point.
This can then be used to detect the distance to the closest object in front of us.

- [Lidar Distance Utility](#lidar-distance-utility)
  - [Configuration](#configuration)
    - [Example](#example)

## Configuration

The possible params for this node are summed up in the following table:

| Parameter         | Description                                           | Default value                     |
|-------------------|-------------------------------------------------------|-----------------------------------|
| min_x             | Minimum x value of the coordinate                     | `np.inf`                          |
| max_x             | Maximum x value of the coordinate                     | `np.inf`                          |
| min_y             | Minimum y value of the coordinate                     | `np.inf`                          |
| max_y             | Maximum y value of the coordinate                     | `np.inf`                          |
| min_z             | Minimum z value of the coordinate                     | `np.inf`                          |
| max_z             | Maximum z value of the coordinate                     | `np.inf`                          |
| point_cloud_topic | Topic to publish the filtered PointCloud2             | `/carla/hero/<namespace>filtered` |
| range_topic       | Topic to publish the Range                            | `/carla/hero/<namespace>_range`   |
| source_topic      | Topic on which the PointCloud2 messages are read from | `/carla/hero/LIDAR`               |

❗️Important:

The LIDAR sensor is mounted in a height of 1.7m (height of the car). Thereby, if you want all points
starting from 20cm above the ground you have to set min_z = -1.5.

The meaning of the x and y values is described by the following image:

![lidar filter](../assets/lidar_filter.png)

### Example

```xml
<node pkg="perception" type="lidar_distance.py" name="lidar_distance" output="screen">
  <param name="max_y" value="1.5"/>
  <param name="min_y" value="-1.5"/>
  <param name="min_x" value="2.5"/>
  <param name="min_z" value="-1.5"/>
  <param name="max_z" value="0"/>
  <param name="point_cloud_topic" value="/carla/hero/LIDAR_filtered"/>
  <param name="range_topic" value="/carla/hero/LIDAR_range"/>
</node>
```
