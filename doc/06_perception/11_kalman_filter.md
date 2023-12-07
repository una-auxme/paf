# Kalman Filter

**Summary:** [kalman_filter.py](.../code/perception/src/kalman_filter.py):

The Kalman Filter node is responsible for filtering the location and heading data, by using an IMU and GNSS sensor together with the carla speedometer.

---

## Author

Robert Fischer

## Date

03.12.2023

## Prerequisite

---
<!-- TOC -->
- [Kalman Filter](#kalman-filter)
  - [Author](#author)
  - [Date](#date)
  - [Prerequisite](#prerequisite)
  - [Getting started](#getting-started)
  - [Description](#description)
    - [1. Predict](#1-predict)
    - [2. Update](#2-update)
    - [3. Publish Data](#3-publish-data)
    - [Inputs](#inputs)
    - [Outputs](#outputs)
<!-- TOC -->

---

## Getting started

Right now the Node does not work correctly. It creates topics to publish to, but doesn't yet.
This will be fixed in [#106](https://github.com/una-auxme/paf23/issues/106)

Uncomment the kalman_filter.py node in the [perception.launch](.../code/perception/launch/perception.launch) to start the node.
You can also uncomment the rqt_plots that seem useful to you.
No extra installation needed.

---

## Description

Sources to understand the topic better:

[Visally Explained Kalman Filters](https://www.youtube.com/watch?v=IFeCIbljreY&ab_channel=VisuallyExplained)

[Understand & Code Kalman Filters](https://www.youtube.com/watch?v=TEKPcyBwEH8&ab_channel=CppMonk)

Stackoverflow and other useful sites:

[1](https://stackoverflow.com/questions/47210512/using-pykalman-on-raw-acceleration-data-to-calculate-position),
[2](https://robotics.stackexchange.com/questions/11178/kalman-filter-gps-imu),
[3](https://stackoverflow.com/questions/66167733/getting-3d-position-coordinates-from-an-imu-sensor-on-python),
[4](https://github.com/Janudis/Extended-Kalman-Filter-GPS_IMU)

This script implements a Kalman Filter. It is a recursive algorithm used to estimate the state of a system that can be modeled with linear equations.
This Kalman Filter uses the location provided by a GNSS sensor (by using the current_pos provided by the [Position Publisher Node](.../code/perception/src/Position_Publisher_Node.py))
the orientation and angular velocity provided by the IMU sensor and the current speed in the headed direction by the Carla Speedometer.

```Python
The state vector X is defined as:
            [initial_x],
            [initial_y],
            [yaw],
            [v_hy] in direction of heading hy,
            [a_hy] in direction of v_hy (heading hy),
            [omega_z],
The state transition matrix F is defined as:
    A = I
    I: Identity Matrix
The measurement matrix H is defined as:
    H = [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 1]
The process covariance matrix Q is defined as:
    Q = np.diag([0.005, 0.005, 0.001, 0.0001])
```

Then 3 Steps are run in the frequency of the `control_loop_rate`:

### 1. Predict

```Python
    # Update the old state and covariance matrix
    self.x_old_est[:, :] = np.copy(self.x_est[:, :])
    self.P_old_est[:, :] = np.copy(self.P_est[:, :])

    # Predict the next state and covariance matrix
    self.x_pred = self.A @ self.x_est[:]  # + B @ v[:, k-1] + u
    self.P_pred = self.A @ self.P_est[:, :] @ self.A.T + self.Q
```

### 2. Update

```Python
    z = np.concatenate((self.z_gps[:], self.z_imu[:]))  # Measurementvector
    y = z - self.H @ self.x_pred  # Measurement residual
    S = self.H @ self.P_pred @ self.H.T + self.R  # Residual covariance
    self.K[:, :] = self.P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman gain
    self.x_est[:] = self.x_pred + self.K[:, :] @ y  # State estimate
    # State covariance estimate
    self.P_est[:, :] = (np.eye(6) - self.K[:, :] @ self.H) @ self.P_pred
```

### 3. Publish Data

```Python
    # Publish the kalman-data:
    self.publish_kalman_heading()
    self.publish_kalman_location()
```

As stated before, the script does not publish viable data yet, which has to be fixed.
This means the predict and update steps might not be correct, because it wasn't possible to test it yet.

### Inputs

This node subscribes to the following needed topics:

- IMU:
  - `/carla/{role_name}/Ideal_IMU` ([IMU](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
- GPS:
  - `/carla/{role_name}/Ideal_GPS` ([NavSatFix](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))
- current agent position:
  - `/paf/{role_name}/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Carla Speed:
  - `/carla/" + self.role_name + "/Speed` CarlaSpeedometer

### Outputs

This node publishes the following topics:

- Kalman Heading:
  - `/paf/{role_name}/kalman_heading` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- Kalman Position:
  - `/paf/{self.role_name}/kalman_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
