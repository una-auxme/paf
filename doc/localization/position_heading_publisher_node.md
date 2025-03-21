# The position_heading_publisher_node

**Summary:** This node forwards the position and heading estimated by the chosen filter to the topics `current_pos` (location of the car) and `current_heading` (orientation of the car around the z-axis).
Several nodes from Planning and Acting use that information.

The node is also used to extract relevant information
from raw data and pass this information on to a different topic so it can be used by the filter nodes more easily.
For example the GPS data consisting of latitude, longitude and altitude is transformed into x/y/z coordinates: /carla/hero/GPS &rarr; /paf/hero/unfiltered_pos

- [Usage](#usage)
- [Overview of the code](#overview-of-the-code)
  - [Inputs](#inputs)
  - [Outputs](#outputs)
  - [Functions regarding the heading](#functions-regarding-the-heading)
  - [Functions regarding the position](#functions-regarding-the-position)

## Usage

There are currently 4 filters to choose from:

- [Extended Kalman Filter](extended_kalman_filter.md) (EKF)
- (Linear) [Kalman Filter](kalman_filter.md) (Kalman)
- Running Average (RunningAvg)
- None

For more information on the different Kalman Filters click the links above.
The Running Average Filter calculates the average of the last few measurements and uses the result as the current estimate.
If you do not want to use a filter and simply make use of the raw sensor data then the None "filter" can be selected.

In the [localization.launch](https://github.com/una-auxme/paf/blob/main/code/localization/launch/localization.launch) file the filter to be used is set.
You only need to set the `filter` argument accordingly, like shown in the following image.

![Filter choice](../assets/localization/filter_choice.jpeg)

Please note, that there is no Running Average Filter implemented for the estimation of the heading.
If this filter is chosen, the position is estimated using the Running Average and for the heading the None "filter" (so raw data) is used.

The Running Average Filter as well as the None "filter" are implemented directly in the position_heading_publisher_node.
All other filters should be implemented in their own file(s) publishing their estimated position / heading to their own topics.
The position_heading_publisher_node can then forward the state estimation to the topics `current_pos` and `current_heading`.

If you want to add a new filter, please refer to [README.md](README.md#adding-a-new-filter)

This modular design visualized in the image above makes it possible for multiple filter nodes to be running and only the data produced by one filter is published as the current position / heading. Otherwise different filters would publish to the same topic (e.g. current_pos) which is not desirable.
However, we choose to save computational power by only launching the node(s), that are needed to estimate the state of the vehicle with the chosen filter.

## Overview of the code

### Inputs

This node subscribes to the following topics:

- OpenDrive Map:
  - `/carla/hero/OpenDRIVE` ([String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- IMU:
  - `/carla/hero/IMU` ([IMU](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- GPS:
  - `/carla/hero/GPS` ([NavSatFix](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- Estimated heading (by the chosen filter):
  - e.g. `/paf/hero/ekf_heading` ([Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- Estimated position (by the chosen filter):
  - e.g. `/paf/hero/ekf_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

### Outputs

This node publishes the following topics:

- Current heading:
  - `/paf/hero/current_heading` ([Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- Current position:
  - `/paf/hero/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Raw (transformed) heading data:
  - `paf/hero/unfiltered_heading` ([Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
  - `/paf/hero/unfiltered_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

### Functions regarding the heading

**`publish_unfiltered_heading`**

This method is called when new IMU data is received.
It publishes unfiltered orientation data as a `Float32` message, so other filters can use it.

If `None` is selected as the filter, it publishes the data as the `current_heading` instead (not recommended).

**`publish_current_heading`**

This method is called when new heading data (from a filter) is received.
The published data has the Message Type `Float32`.

### Functions regarding the position

**`publish_running_avg_pos`**

This method implements the Running Average Filter and is only called if this filter is selected.
When new GPS data is received, the function calculates the average position over the last few measurements and then publishes the result as the current position.

**`publish_filter_pos_as_current_pos`**

This method is called when new filter data (for example from the EKF) is received. The function publishes the estimated position as the current position.

**`publish_unfiltered_gps`**

This method is called when new GPS data is received.
It only transforms the latitude/longitude/altitude into x/y/z coordinates.
Then this position data is published as a `PoseStamped` message, so filters (like the Kalman Filter) can use it.

If `None` is selected as the Filter, it publishes the data as the `current_pos` instead (not recommended).

**`get_geoRef`**

Reads the reference values for latitude and longitude from the carla OpenDriveMap.
Otherwise we could not calculate the Global Coordinate System.
