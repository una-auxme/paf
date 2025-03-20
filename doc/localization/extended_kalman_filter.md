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



### Possible improvements

Currently an Extended Kalman Filter (EKF) is used to estimate the state of the vehicle.
It is likely that even better performance could be achieved by using an Unscented Kalman Filter (UKF).
The robot_localization package does provide an implementation however right now we chose not to use it as it would need more computational power and the EKF works well enough.

The localization of the vehicle could further be improved by combining the current estimate of the position with data generated by image processing.
Using the vision node and the lidar distance node it is possible to calculate the distance to detected objects (for details see [distance_to_objects.md](./distance_to_objects.md)).
Objects such as signals (traffic signs, traffic lights, ...) have a specified position in the OpenDrive map.
For details see: (Disclaimer: the second source belongs to a previous version of OpenDrive but is probably still applicable)

- [source_1](https://www.asam.net/standards/detail/opendrive/) and
- [source_2](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4422&token=e590561f3c39aa2260e5442e29e93f6693d1cccd#top-016f925e-bfe2-481d-b603-da4524d7491f) (menu point "12. Signals")

The knowledge of the map could be combined with the calculated distance to a detected object. For a better understanding look at the following example:
The car is driving on "road1". This road is 100 meters long and there is a traffic light in the middle of it.
If the program detects a traffic light next to the road with a distance of 20 meters this suggests that the vehicle is 30 meters down "road 1".
That information could be used to refine the position estimation of the vehicle.




# Extended Kalman Filter

**Summary:**

The Extended Kalman Filter node will be responsible for filtering the position and heading data. Currently the used sensors are an IMU and GNSS sensor together with the CARLA speedometer.

The position is three-dimensional, but we assume that the car is only driving in a plane right now. That is why the z position and is not estimated by the Kalman Filter but instead is currently calculated using a rolling average.

Currently the state vector of the car is calculated with the data from three different sensors.
The goal is to also include sensor fusion in the sense that the heading as well as the acceleration will be calculated using different sensor data.
The heading will be derived from the angle of the front wheels (which can be derived from data published by Acting) and the acceleration will be calculated using the throttle (which can be derived from data published by Acting).

This file covers the following topics:
- [Theory on linear Kalman Filters](#theory-on-linear-kalman-filters)
  - [1. Prediction (KF)](#1-prediction-kf)
  - [2. Correction (KF)](#2-correction-kf)
- [Implementation of the linear Kalman Filter](#implementation-of-the-linear-kalman-filter)
- [Extending the model](#extending-the-model)
  - [1. Prediction (EKF)](#1-prediction-ekf)
  - [2. Correction (EKF)](#2-correction-ekf)
- [Sensor fusion](#sensor-fusion)
  - [Sensor fusion for the position](#sensor-fusion-for-the-position)
  - [Sensor fusion for the heading](#sensor-fusion-for-the-heading)
  - [Splitting the state vector](#splitting-the-state-vector)

## Theory on linear Kalman Filters

A Kalman Filter is used to estimate the state of a system. In this case we are interested in the position and heading of the vehicle.
The filter uses discrete time intervals which means that the state is estimated for time steps like $k = 1$, $k = 2$, $k = 3$ and so on. (The letter $k$ is used to make the distinction to the continuous time $t$.)
In the following the time difference between these steps is called $\Delta t$.

For simpler calculations we assume that the car only drives in a plane, so the position only consists of the x- and y-coordinates.
We also want to know the rotation of the vehicle in the x-y-plane, so the rotation around the z-axis. This is also known as yaw but in the following we will call it the heading.

You could think that the state vector should therefore only consist of three components: the x-position, the y-position and the heading.
But to calculate these entries we need some other variables (like the velocity) which are also included in the state vector.

The Filter that was used up until now is a (linear) [Kalman Filter](kalman_filter.md). The state vector looks like this:

$$
\vec{x}_{state} =
\begin{bmatrix}
  x\\
  y\\
  v_x\\
  v_y\\
  \varphi\\
  \dot{\varphi}\\
\end{bmatrix}
$$

The meaning of the symbols used in the vector and their calculation is explained in the following table:

$$
\begin{array}{ c l l }
  \text{symbol} & \text{description} & \text{calculation} \\
  x & \text{x-position} & x + v_x \cdot \Delta t \\
  y & \text{y-position} & y + v_y \cdot \Delta t \\
  v_x & \text{velocity in x-direction} & v_x \text{ (const.)}\\
  v_y & \text{velocity in y-direction} & v_y \text{ (const.)}\\
  \varphi & \text{heading / rotation around z-axis} & \varphi +  \dot{\varphi} \cdot \Delta t \\
  \dot{\varphi} & \text{angular velocity around z-axis} & \dot{\varphi} \text{ (const.)}\\
\end{array}
$$

After an initialization (of $x^+(0)$ and $P^+(0)$ ) the algorithm for the Kalman Filter consists of two steps which get repeated over and over:

### 1. Prediction (KF)

The state and its covariance matrix (the uncertainty of the state) are estimated. The formulas can be seen below (assuming there are no external inputs).

$x^-(k) = A(k-1) x^+(k-1)$

$P^-(k) = A(k-1) P^+(k-1)A^T(k-1) + Q(k-1)$

The matrix $A$ is a matrix that describes how the system operates by itself. So given the state vector and the calculation formulas for the entries the matrix would look like this:

$$
A =
\begin{bmatrix}
    1 & 0 & \Delta t & 0 & 0 & 0 \\
    0 & 1 & 0 & \Delta t& 0 & 0 \\
    0 & 0 & 1 & 0 & 0 & 0\\
    0 & 0 & 0 & 1 & 0 & 0 \\
    0 & 0 & 0 & 0 & 1 & \Delta t \\
    0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

The matrix $Q$ is the process noise covariance matrix. This means that it represents how much you trust your model.
The entries in the matrix are set by you and can be adjusted.
If the entries in the matrix are big, this means that there's a lot of uncertainty of the states of the model. As a consequence you trust the measurements more.
If the entries in the matrix are small, this means that you trust the states in the model. As a consequence you trust the model more and the measurements less.

### 2. Correction (KF)

The estimated state gets compared to the measured values. The state and its covariance matrix are corrected according to the error. The formulas are as follows (assuming there are no external inputs).

$K(k) = P^-(k)C^T(k) [R(k) + C(k)P^-(k)C^T(k)]^{-1}$

$x^+(k) = x^-(k) + K(k) [y(k) - C(k)x^-(k)]$

$P^+(k) = [I - K(k)C(k)]P^-(k) [I - K(k)C(k)]^T + K(k)R(k)K^T(k)$

The matrix $K$ is known as the Kalman gain.

The matrix $C$ is the observation matrix. It defines how the state vector is transformed to obtain the predicted measurement​. This ends up being an identity matrix (just like the matrix $I$).

The matrix $R$ is the covariance matrix of the (measurement) noise. Its entries are often estimated and derived from the data sheet of the sensors.

The vector $y$ consists of the measurements.

## Implementation of the linear Kalman Filter

The implementation of this (linear) Kalman Filter can be found in the file [kalman_filter.py](../../code/perception/src/kalman_filter.py)

The [viz.py](../../code/perception/src/experiments/Position_Heading_Datasets/viz.py) file contains helpful functions to visualize and compare the differences between the true position/heading data (taken from the CARLA simulator), the raw data and the filtered data.

The current results from the filter can be seen in the following three pictures.

The first picture shows the x position of the car. The green line represents the true position while the orange line depicts the unfiltered data coming out from the sensor.
The purple line shows the result of the test filter which in this case is the Kalman filter.
It can be seen that the filter follows the measurements too much which results in huge errors (almost 5 meters in the example picture).

![kalman_filter_x_pos](../../doc/assets/localization/kalman_filter_x_pos.png)

The second picture shows the y position of the car. The colors are the same as above.
The filter follows the measurements too much again. Huge errors can also be noticed here (almost 5 meters in the example picture).

![kalman_filter_y_pos](../../doc/assets/localization/kalman_filter_y_pos.png)

The last picture shows the heading of the car.
The dotted red line represents the true heading of the car while the current heading (the green line) is the data produced by the Kalman filter.
It can be seen that the filtered data is really close to the true heading so the focus for the next developed filter should be the improvement of the position data rather than the heading.

![kalman_filter_heading](../../doc/assets/localization/kalman_filter_heading.png)

## Extending the model

Let's look closer at the calculation of the velocity in the x and y direction.
It is currently assumed that we can measure the velocity in the respective directions independently.

In truth the CARLA Speedometer (the sensor responsible for the velocity measurements) only outputs the velocity ($v$) of the vehicle.
That means that $v_x$ and $v_y$ need to be calculated in the following manner:

$v_x = v \cdot cos(\varphi)$

$v_y = v \cdot sin(\varphi)$

This actually creates a problem because we need to use non-linear functions.
Furthermore the velocities are dependent on the heading angle $\varphi$ which is also an entry in the state vector.

As a result the matrix $A$ can not be used like before anymore.
Instead we linearize the current mean and its covariance.
This means that we will replace the current matrix $A$ with a Jacobian matrix.

Currently it is assumed that the vehicle always drives at a constant velocity.
In a normal car this assumption is often not satisfied.
So to make the model more accurate we will add the acceleration of the vehicle to the equations.

The measurements for the linear acceleration in each direction (x/y/z) are provided by the IMU sensor.
Just like with the velocity we ignore the acceleration in the z direction for now and continue to assume that the vehicle drives in a plane.

It is worth to mention that the velocity is no longer divided into two state vector entries $v_x$ and $v_y$ but is combined into the entry $v$.

Now the state vector will look like this:

$$
\vec{x}_{state} =
\begin{bmatrix}
  x\\
  y\\
  v\\
  a\\
  \varphi\\
  \dot{\varphi}\\
\end{bmatrix}
$$

The meaning of the symbols used in the vector and their calculation is explained in the following table:

$$
\begin{array}{ c l l c }
  \text{symbol} & \text{description} & \text{calculation} & \text{function} \\
  x & \text{x-position} & x + v \cdot cos(\varphi) \cdot \Delta t + \frac{1}{2} \cdot a \cdot cos(\varphi) \cdot {\Delta t}^2 & f_1 \\
  y & \text{y-position} & y + v \cdot sin(\varphi) \cdot \Delta t + \frac{1}{2} \cdot a \cdot sin(\varphi) \cdot {\Delta t}^2 & f_2 \\
  v & \text{velocity} & v + a \cdot \Delta t & f_3 \\
  a & \text{acceleration} & \sqrt{a_x^2 + a_y^2} & f_4 \\
  \varphi & \text{heading / rotation around z-axis} & \varphi + \dot{\varphi} \cdot \Delta t & f_5 \\
  \dot{\varphi} & \text{angular velocity around z-axis} & \dot{\varphi} & f_6 \\
\end{array}
$$

As the filter is now non-linear it is called an **Extended Kalman Filter**.
However the basic steps of a Kalman Filter are still maintained even though the formulas are slightly different.

After an initialization (of $x^+(0)$ and $P^+(0)$ ) the algorithm consists of the same two steps as before which get repeated over and over:

### 1. Prediction (EKF)

$x^-(k) = f_D(x^+(k-1))$

$P^-(k) = A(k-1) P^+(k-1)A^T(k-1) + Q(k-1)$ with $A(k-1) = \frac{\delta f_D}{\delta x} \vert_{x^+(k-1)}$

The formula for the matrix $A$ might look a bit intimidating but like mentioned before we just need to calculate the Jacobi matrix.

$$
A =
\begin{bmatrix}
    \frac{\delta f_1}{\delta x} & \frac{\delta f_1}{\delta y} & \frac{\delta f_1}{\delta v} & ... & \frac{\delta f_1}{\delta \dot{\varphi}} \\
    ... & ... & ... & ... & ... \\
    \frac{\delta f_6}{\delta x} & \frac{\delta f_6}{\delta y} & \frac{\delta f_6}{\delta v} & ... & \frac{\delta f_6}{\delta \dot{\varphi}} \\
\end{bmatrix}
$$

$$
A = 
\begin{bmatrix}
    1 & 0 & cos(\varphi) \cdot \Delta t & \frac{1}{2} \cdot cos(\varphi) \Delta t^2 & -v \cdot \Delta t sin(\varphi) - \frac{1}{2} \cot a \cdot \Delta t^2 \cdot sin(\varphi) & 0 \\
    0 & 1 & sin(\varphi) \cdot \Delta t & \frac{1}{2} \cdot sin(\varphi) \Delta t^2 & v \cdot \Delta t cos(\varphi) + \frac{1}{2} \cot a \cdot \Delta t^2 \cdot cos(\varphi) & 0 \\
    0 & 0 & 1 & \Delta t & 0 & 0 \\
    ... & ... & ... & ... & ... & ... \\
    0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

### 2. Correction (EKF)

$K(k) = P^-(k)C^T(k) [R(k) + C(k)P^-(k)C^T(k)]^{-1}$ with $C(k) = \frac{\delta g}{\delta x} \vert_{x^-(k)}$

$x^+(k) = x^-(k) + K(k) [y(k) - g(x^-(k))]$

$P^+(k) = [I - K(k)C(k)]P^-(k) [I - K(k)C(k)]^T + K(k)R(k)K^T(k)$

The matrix $C$ describes how the state vector is transformed to get the predicted measurement. In our case this results in an identity matrix because the function $g$ basically has no effect and just passes along the state entries.

## Sensor fusion

In an autonomous vehicle we often fuse the data of multiple sensors to add redundancy and certainty.
If two sensors are measuring the same thing, for example a distance, then incomplete or noisy data can be compensated by fusing those two measurements.

For this purpose we can use a Kalman filter.

Like before we will estimate a state and its corresponding uncertainty (via a covariance matrix).
As an example you could use the fusion of two Lidar sensors that measured the distance to a pedestrian ([source](https://www.thinkautonomous.ai/blog/sensor-fusion/)).

![sensor_fusion_example](../../doc/assets/localization/sensor_fusion_example.png)

The first Lidar measures the pedestrian at 10 meters distance while the second detects it 10.8 meters away.
If we consider both sensors and their respective uncertainties we will get a better estimation on where the pedestrian is really located.

In a Kalman filter we have a constant cycle of prediction and correction.
In the prediction step we estimate the state only based on the model.
Then in the correction step a measurement from the sensor is taken and the difference between the prediction and measurement as well as the uncertainties from the measurement / process noise are taken into account to update the current state estimation.
Because we combined the knowledge of the model and measurement the uncertainty of this updated state is then far less than before.

A state (estimate, measurement or updated state) can be represented by a Gaussian distribution so the Kalman filter cycle can be visualized like this ([source](https://www.thinkautonomous.ai/blog/sensor-fusion/)):

![kalman_cycle_visualization](../../doc/assets/localization/kalman_cycle_visualization.png)

So if we want to improve the estimation and its certainty by using multiple sensors, a new prediction and correction cycle is run every time new sensor data arrives.

In the following picture ([source](https://www.thinkautonomous.ai/blog/sensor-fusion/)) you can see an example of improving the prediction of a system state (in this case position and velocity) by using a Radar and Lidar sensor.

![sensor_fusion_cycle](../../doc/assets/localization/sensor_fusion_cycle.png)

The advantage of sensor fusion is that more data can be used which generally improves the estimation.

It is important to note that for all sensors the same values are calculated which means that the state vector stays the same.

### Sensor fusion for the position

The x and y position of the vehicle is dependent on the acceleration of the car.
Instead of taking the linear acceleration data provided by the IMU sensor we could use the throttle.
The applied throttle is controlled by Acting is published on a topic.

The possible formula ([source](https://robotics.stackexchange.com/questions/11178/kalman-filter-gps-imu)) for the transformation from throttle to acceleration is: $a = \frac{c(throttle) - v}{\tau}$

The parameter $\tau$ is the time constant and $c$ is a value that scales the throttle to a speed.
Setting the parameters would require creating a step response from the car for setting the throttle to a max.
Realising this might prove itself quite difficult.

### Sensor fusion for the heading

Currently the heading ($\varphi$) is calculated by "integrating" the angular velocity around the z axis ($\dot{\varphi}$). The angular velocities are measured by the IMU sensor.

An alternative way is to calculate the heading via a 2D bicycle model which can be seen as a simplification of a car (with only two wheels).
For the model a reference point needs to be set.
For now it seems the easiest to place it at the center of the rear axle.

An illustration of the model can be seen in the following picture ([source](https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html)).

![bicycle_model](../../doc/assets/localization/bicycle_model.png)

ICR stands for Instantaneous Center of Rotation.
The angle $\theta$ describes the heading, $\delta$ is the steering angle (which can be derived from data published by Acting) and $\omega$ stands for the angular velocity (around the z axis).
Another important parameter is $L$ which is the wheelbase (so the distance between the rear and front axle).

From this setup we can derive a formula for the angular velocity: $\dot{\varphi} \leftarrow \frac{v}{L} \cdot tan(\delta)$

The heading can then be calculated like before: $\varphi \leftarrow \varphi + \dot{\varphi} \cdot \Delta t$

Because the angular velocity is dependent on the steering angle $\delta$ in a non-linear manner, the angle $\delta$ should be added as a state vector entry.
