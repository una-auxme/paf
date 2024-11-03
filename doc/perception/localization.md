# Localization

There are currently three nodes working together to localize the vehicle:

- [position_heading_publisher_node](./position_heading_publisher_node.md)
- [kalman_filter](./kalman_filter.md)
- [coordinate_transformation](./coordinate_transformation.md)

The [position_heading_filter_debug_node](./position_heading_filter_debug_node.md) node is another useful node but it does not actively localize the vehicle.
Instead it makes it possible to compare and tune different filters and the [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py) file is the recommended way to visualizes the results.

## position_heading_publisher_node

The [position_heading_publisher_node](./position_heading_publisher_node.md) node is responsible for publishing the position and heading of the car. It publishes raw as well as already filtered data. The goal of filtering the raw data is the elimination / reduction of noise.

### Published / subscribed topics

The following topics are therefore published by this node:

- `unfiltered_pos` (raw data, subscribed to by filter nodes e.g. kalman_filter)
- `unfiltered_heading` (raw data, subscribed to by filter nodes e.g. kalman_filter)
- `current_pos` (filtered data, position of the car)
- `current_heading` (filtered data, orientation of the car around the z-axis)

To gather the necessary information for the topics above the node subscribes the following topics:

- OpenDrive (map information)
- IMU (Inertial Measurement Unit)
- GPS
- the topic published by the filter that is used (e.g. kalman_pos)

As you can see the node first subscribes the filtered data (e.g. kalman_pos) and then publishes this data as the current position / heading. It merely passes along the data.

This makes it possible for multiple filter nodes to be running and only the data produced by one filter is published as the current position / heading. Otherwise different filters would publish to the same topic (e.g. current_pos) which is not desirable.

### Available filters

The filter to be used is specified in the [perception.launch](../../code/perception/launch/perception.launch) file.

The currently available filters are as follows:

- position filter:
  - Kalman (Kalman Filter)
  - RunningAvg (Running Average Filter)
  - None (No Filter)
- heading filter:
  - Kalman (Kalman Filter)
  - None (No Filter)
  - Old (heading is calculated the WRONG way, for demonstration purposes only)

To use a certain / new filter two files need to be updated:

- First make sure that in the [perception.launch](../../code/perception/launch/perception.launch) file:
  - the node of the filter you want to use is included (e.g. kalman_filter.py)
  - the `pos_filter` / `heading_filter` arguments are set accordingly (e.g. "Kalman") in the code of the position_heading_publisher_node node

- Then the according subscriber and publisher need to be added in the init function of the position_heading_publisher_node.py file.

For further details on the position_heading_publisher_node node click [here](./position_heading_publisher_node.md).

## kalman_filter

The currently used filter is the (linear) Kalman Filter. It is responsible for filtering the location and heading data so the noise can be eliminated / reduced.

### Published / subscribed topics

Therefore the published topics are:

- `kalman_pos` (filtered position of the vehicle)
- `kalman_heading` (filtered heading of the vehicle)

The variables to be estimated are put together in the state vector. It consists of the following elements:

- `x` (position on the x-axis)
- `y` (position on the y-axis)
- `v_x` (velocity in the x-direction)
- `v_y` (velocity in the y-direction)
- `yaw` (orientation, rotation around z-axis)
- `omega_z` (angular velocity)

The z-position is currently not estimated by the Kalman Filter and is calculated using the rolling average.

The x-/y-position is measured by the GNSS sensor. The measurement is provided by the unfiltered_pos topic.

The velocity in x-/y-direction can be derived from the speed measured by the Carla Speedometer in combination with the current orientation.

To get the orientation and angular velocity of the vehicle the data provided by the IMU (Inertial Measurement Unit) sensor is used.

### Possible improvements

In earlier experiments it was shown that the Kalman Filter performs better than using the rolling average or the unfiltered data. But it seems likely that further improvements can be made.

For further details on the current implementation of the kalman_filter node click [here](./kalman_filter.md).

Currently the model assumes that the vehicle drives at a constant speed. Adding acceleration in the model (as proposed [here](https://www.youtube.com/watch?v=TEKPcyBwEH8)) could ultimately improve the filter.

It is also likely to achieve a better performance by using a non-linear filter like the Extended or Unscented Kalman Filter.
The latter is the most generic of the three options as it does not even assume a normal distribution of the system but it is also the most complex Kalman Filter.

The localization of the vehicle could further be improved by combining the current estimate of the position with data generated by image processing.
Using the vision node and the lidar distance node it is possible to calculate the distance to detected objects (for details see [distance_to_objects.md](./distance_to_objects.md).
Objects such as signals (traffic signs, traffic lights, ...) have a specified position in the OpenDrive map.
For details see: (Disclaimer: the second source belongs to a previous version of OpenDrive but is probably still applicable)

- [source_1](https://www.asam.net/standards/detail/opendrive/) and
- [source_2](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4422&token=e590561f3c39aa2260e5442e29e93f6693d1cccd#top-016f925e-bfe2-481d-b603-da4524d7491f) (menu point "12. Signals")

The knowledge of the map could be combined with the calculated distance to a detected object. For a better understanding look at the following example:
The car is driving on "road1". This road is 100 meters long and there is a traffic light in the middle of it.
If the program detects a traffic light next to the road with a distance of 20 meters this suggests that the vehicle is 30 meters down "road 1".
That information could be used to refine the position estimation of the vehicle.

## coordinate_transformation

The [coordinate_transformation](./coordinate_transformation) node provides useful helper functions such as quat_to_heading which transforms a given quaternion into the heading of the car.

This node is used by the [position_heading_publisher_node](./position_heading_publisher_node) node as well as the [kalman_filter](./kalman_filter) node. Both nodes use the node for its quat_to_heading function and its CoordinateTransformer class.

The node is not fully documented yet but for further details on the quat_to_heading function click [here](./coordinate_transformation.md).

## position_heading_filter_debug_node

This node processes the data provided by the IMU and GNSS so the errors between the is-state and the measured state can be seen.
To get the is-state the Carla API is used to get the real position and heading of the car.
Comparing the real position / heading with the position / heading estimated by a filter (e.g. Kalman Filter) the performance of a filter can be evaluated and the parameters used by the filter can be tuned.

The recommended way to look at the results is using the mathplotlib plots provided by the [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py) file even though they can also be shown via rqt_plots.

Because the node uses the Carla API and therefore uses the ground truth it should only be used for combaring and tuning filters and not for any other purposes.
It might be best to remove this node before submitting to the official leaderboard because otherwise the project could get disqualified.

For more details on the node see [position_heading_filter_debug_node](./position_heading_filter_debug_node.md) and [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py).
