# position_heading_filter_debug_node.py

**Summary:** [position_heading_filter_debug_node.py](../../code/perception/src/position_heading_filter_debug_node.py):

The position_heading_filter_debug_node node is responsible for collecting sensor data from the IMU and GNSS and process the data in such a way, that it shows the errors between the real is-state and the measured state.
The data is the shown in multiple rqt_plots.

---

## Author

Robert Fischer

## Date

03.12.2023

## Prerequisite

---
<!-- TOC -->
- [sensor\_filter\_debug.py](#position_heading_filter_debugpy)
  - [Author](#author)
  - [Date](#date)
  - [Prerequisite](#prerequisite)
  - [Getting started](#getting-started)
  - [Description](#description)
    - [Inputs](#inputs)
    - [Outputs](#outputs)
<!-- TOC -->

---

## Getting started

Uncomment the position_heading_filter_debug_node.py node in the [perception.launch](../../code/perception/launch/perception.launch) to start the node.
You can also uncomment the rqt_plots that seem useful to you, or create your own ones from the data published.
You have to add the following sensors to the sensors inside the [dev_objects.json](../../code/agent/config/dev_objects.json):

```json
{
          "type": "sensor.other.gnss",
          "id": "Ideal_GPS",
          "spawn_point": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.60
          },
          "noise_alt_stddev": 0.0,
          "noise_lat_stddev": 0.0,
          "noise_lon_stddev": 0.0,
          "noise_alt_bias": 0.0,
          "noise_lat_bias": 0.0,
          "noise_lon_bias": 0.0
        },
        {
          "type": "sensor.other.imu",
          "id": "Ideal_IMU",
          "spawn_point": {
            "x": 0.7,
            "y": 0.4,
            "z": 1.60,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
          },
          "noise_accel_stddev_x": 0.0,
          "noise_accel_stddev_y": 0.0,
          "noise_accel_stddev_z": 0.0,
          "noise_gyro_stddev_x": 0.0,
          "noise_gyro_stddev_y": 0.0,
          "noise_gyro_stddev_z": 0.0
        },
        {
          "type": "sensor.other.radar",
          "id": "Ideal_RADAR",
          "spawn_point": {
            "x": 0.7,
            "y": 0.4,
            "z": 1.60,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 45.0
          },
          "horizontal_fov": 30.0,
          "vertical_fov": 30.0,
          "points_per_second": 1500,
          "range": 100.0
        }
```

You also have to launch it in dev mode, otherwise the added ideal sensors won't work.
No extra installation needed.

---

## Description

Running the node provides you with ideal Sensor topics that can be used to debug your sensor filters by giving you ideal values you should aim for.
Right now only the IMU and the GNSS sensor are available for debug.
Debug for the RADAR and LIDAR hasn't been implemented yet.

An Example of Location Error Output can be seen here:
![Distance from current_pos to ideal_gps_pos (blue) and to carla_pos (red)](../00_assets/gnss_ohne_rolling_average.png)

### Inputs

This node subscribes to the following needed topics:

- OpenDrive Map:
  - `/carla/{role_name}/OpenDRIVE` ([String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)) or `/carla/world_info` ([CarlaWorldInfo](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/#carlaworldinfomsg))
- Ideal_IMU:
  - `/carla/{role_name}/Ideal_IMU` ([IMU](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- Ideal_GPS:
  - `/carla/{role_name}/Ideal_GPS` ([NavSatFix](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- current agent position:
  - `/paf/{role_name}/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- current agent heading:
  - `/paf/{role_name}/current_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- [Carla_API](https://carla.readthedocs.io/en/latest/python_api/)
  - `get_location`

### Outputs

This node publishes the following topics:

- Ideal_IMU:
  - `/carla/{role_name}/Ideal_IMU` ([IMU](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- Ideal_GPS:
  - `/carla/{role_name}/Ideal_GPS` ([NavSatFix](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- Ideal Odometry:
  - `/ideal_odom` ([Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- Ideal current agent position (calculated from ideal GPS):
  - `/paf/{role_name}/idealcurrent_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Ideal current agent heading (calculated from ideal IMU):
  - `/paf/{role_name}/current_heading` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Carla Position:
  - `/paf/{self.role_name}/carla_current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Location Error:
  - `/paf/{self.role_name}/location_error` ([Float32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32MultiArray.html))
- Heading Error:
  - `/paf/{self.role_name}/heading_error` ([Float32](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Ideal Coordinate Publisher (used to debug [#105](https://github.com/una-auxme/paf23/issues/105))
  - Ideal X Publisher:
    - `/paf/{self.role_name}/ideal_x`([Float32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32MultiArray.html))
  - Ideal Y Publisher:
    - `/paf/{self.role_name}/ideal_y`([Float32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32MultiArray.html))
