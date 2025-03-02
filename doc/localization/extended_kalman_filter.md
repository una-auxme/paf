# The Extended Kalman Filter

**Summary:**
The state of the vehicle (position and heading) can be estimated using an Extended Kalman Filter.
The robot_localization package provides an implementation of this filter which fuses IMU, GPS, as well as Odometry data (computed using the speed of the car and its steering angle) to compute an accurate state estimation.

- [robot\_localization](#robot_localization)
- [Architecture](#architecture)
- [Summary of components](#summary-of-components)
  - [localization.launch](#localizationlaunch)
  - [sensor\_covariance\_fusion.py](#sensor_covariance_fusionpy)
  - [odometry\_fusion.py](#odometry_fusionpy)
  - [gps\_transform.py](#gps_transformpy)
  - [current\_state\_publisher.py](#current_state_publisherpy)
- [Configuration files](#configuration-files)
  - [ekf\_config.yaml](#ekf_configyaml)
  - [odometry\_covariances.yaml](#odometry_covariancesyaml)
  - [sensor\_covariances.yaml](#sensor_covariancesyaml)

## robot_localization

The [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package ([GitHub](https://github.com/cra-ros-pkg/robot_localization))
is a ROS-package providing [state_estimation_nodes](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst)
such as an Extended Kalman Filter (EKF) or an Unscented Kalman Filter (UKF).
As the UKF is quite computationally intensive, we decided to use an EKF as it works very well and doesn't take up too many resources.
Because inputs can be defined and added easily, we decided on using this as a framework for our localization algorithm.

## Architecture

There are two EKF Nodes running:

The first node (local_ekf) fuses Odometry data with IMU data in order to track the cars position relative to the odom frame.
The odom frame itself is irrelevant for us, but this follows not only the suggestion provided in the robot localization package but also conforms with [REP 105](https://www.ros.org/reps/rep-0105.html).

The second node (global_ekf) fuses the GPS data with  IMU data in order to calculate the global position.
This node publishes the global to odom frame.

The frames are attached as follows:

global $\rightarrow$ odom $\rightarrow$ hero

When comparing this to [REP 105](https://www.ros.org/reps/rep-0105.html) this correlates to:

map $\rightarrow$ odom $\rightarrow$ base_link

An overview of all different nodes working together can be seen in the following picture.

![Overview EKF](../../doc/assets/localization/overview_ekf.jpeg)

## Summary of components

There are several files in the localization package. The following are a list of the most relevant.

### [localization.launch](../../code/localization/launch/localization.launch)

This launch file launches all appropriate nodes.
Apart from our custom-made nodes (see below) this also launches the two EKF nodes described in [Architecture](#architecture).

### [sensor_covariance_fusion.py](../../code/localization/src/sensor_covariance_fusion.py)

As the IMU and GPS data is provided by Carla without covariance but the EKF nodes rely on the covariances beeing part of the message, this node catches the IMU and GPS messages, adds the appropriate covariances and republishes them.

### [odometry_fusion.py](../../code/localization/src/odometry_fusion.py)

This node calculates the odometry message from the current speed and the current steering angle of the vehicle.
It then publishes this together with appropriate covariances.

### [gps_transform.py](../../code/localization/src/gps_transform.py)

The gps_transform node is a substitute for the [navsat_transform_node](http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html#using-navsat-transform-node) provided in the robot_localization package.
This needed to be custom-made because the navsat_transform_node converts GPS data into UTM-coordinates.
However, Carla assumes WGS coordinates.

### [current_state_publisher.py](../../code/localization/src/current_state_publisher.py)

As most of the current PAF implementation relies not on the tf-data but on custom messages (namely current_position, current_heading) this node listens to the tf-graph and converts it to this data.

## Configuration files

### [ekf_config.yaml](../../code/localization/config/ekf_config.yaml)

With this .yaml file the EKF nodes can be configured.
The first section configures the local EKF, the second one the global EKF.

### [odometry_covariances.yaml](../../code/localization/config/odometry_covariances.yaml)

This file defines the covariance matrices for the Odometry data.

### [sensor_covariances.yaml](../../code/localization/config/sensor_covariances.yaml)

This file contains the covariance matrices for GPS and IMU data.
