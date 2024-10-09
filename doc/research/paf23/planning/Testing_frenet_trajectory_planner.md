# Frenet Optimal Trajectory

**Summary:** This document summarizes the Frenet Optimal Trajectory planner used in the pylot project

---

## Author

Samuel Kühnel

## Date

15.01.2024

## Setup

```git
git clone https://github.com/fangedward/frenet-optimal-trajectory-planner.git
./build.sh
```

## Example Usage

There is a Python wrapper and C++ API. The Python wrapper is located in `FrenetOptimalTrajectory/fot_wrapper.py`.

A test can be run with

```shell
python3 FrenetOptimalTrajectory/fot.py
```

A test python file is also located [here](test_traj.py). The below image was generated using that script.

The orange points represent a possible object and the blue points the old (left) and new (right) trajectory.

![test_trajectory](../../../assets/planning/test_frenet_results.png)

## Inputs

To use the Planner you need a set of waypoints, objects and initial and target velocity. The information needs to be clustered inside a dictionary.

Additionally there are several hyperparameters like maximum values for speed, acceleration, obstacle clearance, road width etc. These are also passed via a dictionary.

Complete list of parameters:

```python
"""
        initial_conditions (dict): dict containing the following items
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            max_speed (float): maximum speed [m/s]
            max_accel (float): maximum acceleration [m/s^2]
            max_curvature (float): maximum curvature [1/m]
            max_road_width_l (float): maximum road width to the left [m]
            max_road_width_r (float): maximum road width to the right [m]
            d_road_w (float): road width sampling discretization [m]
            dt (float): time sampling discretization [s]
            maxt (float): max prediction horizon [s]
            mint (float): min prediction horizon [s]
            d_t_s (float): target speed sampling discretization [m/s]
            n_s_sample (float): sampling number of target speed
            obstacle_clearance (float): obstacle radius [m]
            kd (float): positional deviation cost
            kv (float): velocity cost
            ka (float): acceleration cost
            kj (float): jerk cost
            kt (float): time cost
            ko (float): dist to obstacle cost
            klat (float): lateral cost
            klon (float): longitudinal cos
"""
```

## Decision

We decided not to use the FOT-Planner, as we got too many instable results and decided to implement a simpler version.
