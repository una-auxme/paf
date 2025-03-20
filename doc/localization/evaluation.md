# Evaluating filters

## position_heading_filter_debug_node

The [position_heading_filter_debug_node](./position_heading_filter_debug_node.md) node is another useful node but it does not actively localize the vehicle.
Instead it makes it possible to compare and tune different filters and the [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py) file is the recommended way to visualizes the results.

This node processes the data provided by the IMU and GNSS so the errors between the is-state and the measured state can be seen.
To get the is-state the Carla API is used to get the real position and heading of the car.
Comparing the real position / heading with the position / heading estimated by a filter (e.g. Kalman Filter) the performance of a filter can be evaluated and the parameters used by the filter can be tuned.

The recommended way to look at the results is using the mathplotlib plots provided by the [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py) file even though they can also be shown via rqt_plots.

Because the node uses the Carla API and therefore uses the ground truth it should only be used for combaring and tuning filters and not for any other purposes.
It might be best to remove this node before submitting to the official leaderboard because otherwise the project could get disqualified.

For more details on the node see [position_heading_filter_debug_node](./position_heading_filter_debug_node.md) and [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py).
