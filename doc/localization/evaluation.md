# Evaluating filters

**Summary:** There are some nodes that can be used to evaluate and tune a filter or compare it to an old or new filter.
All nodes needed for this can be found in the [evaluation](https://github.com/una-auxme/paf/tree/main/code/localization/src/evaluation) folder.

**WARNING:** To compare filter estimations to the ground truth, the nodes, that are saving data, are connected to the Carla API.
Therefore, the [evaluation](https://github.com/una-auxme/paf/tree/main/code/localization/src/evaluation) folder should be removed before submitting to the Leaderboard, because otherwise the project could get disqualified.

**Disclaimer:** All of the nodes described here are not launched by default.
Please refer to Section [Starting the nodes](#starting-the-nodes) for an explanation of how the nodes can be started.

- [Starting the nodes](#starting-the-nodes)
  - [Include them in the localization.launch file](#include-them-in-the-localizationlaunch-file)
  - [Start them from a shell](#start-them-from-a-shell)
- [Comparing filters with focus on errors](#comparing-filters-with-focus-on-errors)
  - [Usage](#usage)
  - [position_heading_filter_debug_node.py](#position_heading_filter_debug_nodepy)
    - [Getting started](#getting-started)
    - [Inputs](#inputs)
    - [Outputs](#outputs)
  - [viz.py](#vizpy)
- [Comparing filters with focus on the state](#comparing-filters-with-focus-on-the-state)
  - [Usage](#usage-1)
  - [save_filter_data.py](#save_filter_datapy)
    - [Getting started](#getting-started-1)
    - [Inputs](#inputs-1)
    - [Outputs](#outputs-1)
  - [visualize_filter_comparison.py](#visualize_filter_comparisonpy)

## Starting the nodes

There are two ways you can start the nodes: include them in the localization.launch file or start them from a shell.

### Include them in the [localization.launch](https://github.com/una-auxme/paf/blob/main/code/localization/launch/localization.launch) file

If you start a node this way, it is started by default every time you run the project.

For example the position_heading_filter_debug_node could be included this way.
It is currently commented out in the
[[localization.launch](https://github.com/una-auxme/paf/blob/main/code/localization/launch/localization.launch#L88-L94)] file.
You can also uncomment the rqt_plots that seem useful to you.

Before starting nodes this way, please note the following:

- If nodes are reading from saved .csv files, please make sure that these files exist before running the project.
- If you don't want to save data and therefore create .csv files every time you run the project, please don't start the respective nodes this way.

### Start them from a shell

In a shell connected to ROS (build-agent &rarr; Attach Shell) you can start a node with the following scheme:

`rosrun [package_name] [node_name]`

For example the command could look like this:

`rosrun localization position_heading_filter_debug_node.py`

Before starting nodes this way, please note the following:

- If nodes are reading from saved .csv files, please make sure that these files exist before running the project.

## Comparing filters with focus on errors

The files described below are useful for comparing different filter estimations and their errors.

### Usage

The position_heading_filter_debug_node saves data in .csv files.
Like mentioned in Section [Include them in the localization.launch file](#include-them-in-the-localizationlaunch-file) the data can be plotted using rqt_plots.

However, the recommended way is to look at the results using the [viz.py](#vizpy) file.
This file visualizes the data in matplotlib plots.

### [position_heading_filter_debug_node.py](../../code/localization/src/evaluation/position_heading_filter_debug_node.py)

The position_heading_filter_debug_node compares a test filter to the current filter in regards to its x-, y- and heading-estimation.
It also records the unfiltered position and heading, which is just preprocessed raw sensor data from the IMU and GPS sensor provided by the [position_heading_publisher_node](position_heading_publisher_node.md).
Furthermore, the ground truth is saved, so a general evaluation is possible.

The data is stored in .csv files in a numerically ordered "data_##.csv" format.
The node creates subfolders in the folder `/workspace/code/localization/src/data/position_heading_datasets`.
The data is saved in the corresponding subfolder:

- `/x_error`
- `/y_error`
- `/heading_error`

Below you can see an example for the formatting of such a .csv file:

| Time | Unfiltered | Ideal(Carla) | Current | Test Filter | Unfiltered Error | Current Error | Test Filter Error |
| ---- | ---------- | ------------ | ------- | ----------- | ---------------- | ------------- | ----------------- |
| 0.1  | 10.0       | 10.1         | 10.2    | 10.3        | 0.1              | 0.2           | 0.3               |
| 0.2  | 20.0       | 20.1         | 20.2    | 20.3        | 0.1              | 0.2           | 0.3               |

So in conclusion:
the is-state, the estimated state and the measured state and the corresponding errors can be compared.
This makes it possible for the performance of a filter to be evaluated.

#### Getting started

If you are implementing a new filter and want to evaluate it using this node, you will have to do the following steps:

1. Create a new filter node (if not already done), that publishes two topics: one for its position estimation and one for its heading estimation.
2. Change the topic of the test filter subscribers to your topics. (Currently the test filter EKF is compared to the currenly used filter (also EKF))

![New test filter](/doc/assets/localization/new_test_filter.jpeg)

You can adjust for how long the file saves data.
With the [DATA_SAVING_MAX_TIME variable](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/position_heading_filter_debug_node.py#L26-L26)
you can set for how many seconds in the simulation data is recorded.

If you do not want to save the data in .csv files anymore, you can comment out the
[responsible lines of code](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/position_heading_filter_debug_node.py#L557-L567).
Please note, that in this case the [viz.py](#vizpy) file can no longer be used for visualization and rqt_plots need to be used instead.

#### Inputs

The node gets input from various sources:

- The current filter (EKF):
  - `/paf/hero/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/paf/hero/current_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- The test filter (also EKF):
  - `/paf/hero/ekf_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/paf/hero/ekf_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- The [Carla API](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/position_heading_filter_debug_node.py#L40-L45)
- The preprocessed IMU and GPS sensor data:
  - `/paf/hero/unfiltered_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/paf/hero/unfiltered_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
  
#### Outputs

This node publishes several topics.
They are needed for plotting with rqt_graphs.

- `/paf/hero/carla_current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- `/paf/hero/carla_current_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- `/paf/hero/position_debug` ([Float32MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))
- `/paf/hero/heading_debug` ([Float32MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html))

### [viz.py](../../code/localization/src/evaluation/viz.py)

The viz.py file can visualize the data saved by the position_heading_filter_debug_node.py in various plots.

Before starting this node, make sure that the corresponding .csv files exist first and set the
[`FILE_NUM` variable](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/viz.py#L23-L24)
according to the number of the file you want to visualize.

It can be used to visualize x- and y-position data as well as heading data.

Before you start, configure the main function to your liking.
Comment out unwanted plots and include plots you want to see.

The available plots are:

- Notched Box Graphs
  - For the x-position (using either MSE or MAE for calculating the error)
  - For the y-position (using either MSE or MAE for calculating the error)
  - For the heading (using either MSE or MAE for calculating the error)
- CEP Graphs (for position errors)
- Graphs showing the x- or y-positions
- Graphs showing the heading

All plots compare the test filter with the current filter and the ideal data (ground truth provided by Carla).

You can also find the "best tuned" file using the `plot_best_tuned_file_by_type` method within a specified file range that you want to compare each file with.
This can be useful if you try to tune your filter doing multiple runs and therefore saving multiple files (with the position_heading_filter_debug_node).

The deciding factor for a best tuned file is the lowest IQR or the lowest MSE (Default) which can also be set as an argument.

To use this method you have to change the `FILE_START` and `FILE_END` values at the
[beginning of the file](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/viz.py#L18-L21)
to the range of data you want to compare.

## Comparing filters with focus on the state

The files described below are useful for comparing different filter estimations and their errors.

### Usage

The save_filter_data file saves data in .csv files.
This saved data can be visualized using the visualize_filter_comparison file.

### [save_filter_data.py](../../code/localization/src/evaluation/save_filter_data.py)

The save_filter_data node makes it possible to compare the estimated state of an old filter with that of a new one and the ground truth (provided by the Carla API).

The data is stored in .csv files in a numerically ordered "data_##.csv" format.
The node creates subfolders in the folder `/workspace/code/localization/src/data/filter_comparison`.
The data is saved in the corresponding subfolder:

- `/new_filter_pos`
- `/new_filter_heading`
- `/old_filter_pos`
- `/old_filter_heading`
- `/ground_truth`

Below you can see an example for the formatting of such a .csv file saving position data:

| Time | pos x | pos y | pos z |
| ---- | ---------- | ------------ | ------- |
| 0.1  | 10.0       | 10.1         | 10.2    |
| 0.2  | 20.0       | 20.1         | 20.2    |

An example for the formatting of such a .csv file saving heading data can be seen in the following:

| Time | heading |
| ---- | ---------- |
| 0.1  | 10.0       |
| 0.2  | 20.0       |

Below an example for the formatting of such a .csv file saving ground truth data can be seen:

| Time | pos x | pos y | pos z | heading |
| ---- | ---------- | ------------ | ------- | ------- |
| 0.1  | 10.0       | 10.1         | 10.2    | 10.3    |
| 0.2  | 20.0       | 20.1         | 20.2    | 20.3    |

#### Getting started

If you are implementing a new filter and want to compare it to the old filter using this node, you will have to do the following steps:

1. Create a new filter node (if not already done), that publishes two topics: one for its position estimation and one for its heading estimation.
2. Change the topics of the filter subscribers to the topics you are interested in. (Currently the newest filter is the EKF. Before that the Kalman Filter was used, which is why it is set as the old filter.)

![New test filter](/doc/assets/localization/change_new_and_old_filter.jpeg)

You can adjust for how long the file saves data.
With the
[DATA_SAVING_MAX_TIME variable](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/save_filter_data.py#L26)
you can set for how many seconds in the simulation data is recorded.

#### Inputs

The node gets input from various sources:

- The new filter (EKF):
  - `/paf/hero/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/paf/hero/current_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- The old filter (Kalman Filter):
  - `/paf/hero/kalman_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
  - `/paf/hero/kalman_heading` ([Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))
- The [Carla API](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/save_filter_data.py#L46-L51)
  
#### Outputs

This node publishes no topics.
It only saves data in .csv files.

### [visualize_filter_comparison.py](../../code/localization/src/evaluation/visualize_filter_comparison.py)

The visualize_filter_comparison file can visualize the data saved by the save_filter_data file in several plots.
It can be used to visualize x-, y-position or heading data as well as errors.

Before starting this node, make sure that the corresponding .csv files exist first and set the
[`NEW_FILTER_FILE_NAME`, `OLD_FILTER_FILE_NAME` and `GT_FILE_NAME` variables](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/visualize_filter_comparison.py#L24-L28)
according to the numbers of the files you want to visualize (normally all three should be the same).

Before you start, configure the
[last few lines](https://github.com/una-auxme/paf/blob/2fa0bde45dad9e3b8236c22fa0bbecc1e28a56cc/code/localization/src/evaluation/visualize_filter_comparison.py#L525-L526)
to your liking.
Comment out unwanted plots and include plots you want to see.

The available plots are:

- plot_x_position
- plot_x_error (MSE used)
- plot_y_position
- plot_y_error (MSE used)
- plot_gt_heading

All plots compare the new filter with the old filter and the ideal data (ground truth provided by Carla).
