# Radar Velocity Estimation Issues with Stationary Objects

**Date:** February 02, 2026

## Setup

- **Main-branch + Radar-FOVs**: 130°
- **Radar output tuple**:  
  `(x, y, z, range, velocity, azimuth_angle, elevation_angle)`
- **Velocity definition**:  
  The radar reports **radial relative velocity** (Doppler velocity along the line of sight).

## Problem Description

Stationary objects are sometimes assigned **high velocities**, even though they are not moving.

### Why This Is a Problem

We want to use radar sensors to determine whether an object is **moving or stationary** (binary decision: *Yes / No*).

- Small velocities could be filtered out using a simple threshold (e.g. `< 2 km/h`)
- However, due to incorrect velocity estimation, stationary objects sometimes exceed this threshold and are therefore misclassified as moving

## Observations

- When our ego vehicle is **standing still**, the detected object velocities are also low (`< 2 km/h`)
- **Distant stationary objects** are detected relatively well and are assigned low velocities
- The **closer** a stationary object is to our vehicle, and the **larger the azimuth angle** relative to the radar, the **higher** the detected velocity becomes
- Since the radar reports **relative velocity**, we currently **add ego velocity** to the radar-reported values

## Current Velocity Computation Pipeline

### 1. `perception/perception/radar_node.py`

- For each cluster:
  - Compute the **mean cluster velocity**
  - Create a **2D velocity vector**
  - **Only the x-component is computed**
  - The y-component is **always set to 0**
- Result:
  - Velocity arrows in RViz always point **towards or away from the ego vehicle**

### 2. `mapping/mapping/data_integration.py`

- The **ego speed** is added to the averaged cluster velocity

### 3. `mapping/mapping_common/entity.py`

- Visualization of velocity arrow and text in RViz:

```python
speed_in_ms = self.motion.linear_motion.length()
speed_in_kmh = speed_in_ms * 3.6
motion_text = f"{speed_in_kmh:.2f} km/h"
```

## Suspected Causes of Incorrect Velocities

- **Azimuth angle is ignored**
  - Radar velocity is treated as a pure forward velocity
- **Y velocity component is always set to zero**
  - Radar radial velocity is incorrectly interpreted as absolute longitudinal velocity (`vx`)
- **Multiple radar points per object**
  - Radar detects reflections from different parts of an object (e.g. front and rear of a vehicle)
  - These points can have very different radial velocities
  - Averaging them may significantly distort the resulting cluster velocity

## What Has Been Tested So Far

- **Removing ego-velocity addition**
  - Result: velocities become **even larger**
- **Including azimuth angle to compute x and y velocity components**
  - This leads to incorrect velocity vector directions
  - When `y ≠ 0`, velocity arrows in RViz no longer align with the actual driving direction
  - Previous angle-based velocity correction did **not** solve the issue

## Open Questions / Potential Next Steps

- Would it work better to:
  - Use **only a single radar point per cluster** instead of averaging all points?
- How can the **azimuth angle** be correctly incorporated to reconstruct object velocity from radial velocity?
- Can incorrect velocities be reduced by:
  - **Dynamically adding ego velocity**, e.g.:
    - Project ego velocity onto the radar line-of-sight
    - Add only the **angle-dependent component** (`ego_speed * cos(azimuth)`)

## Summary

The core issue appears to be a **misinterpretation of radial Doppler velocity as absolute longitudinal velocity**,
especially for objects at large azimuth angles and close range. Proper handling of radar geometry, angular dependency,
and ego-motion compensation is likely required to reliably distinguish between moving and stationary objects.
