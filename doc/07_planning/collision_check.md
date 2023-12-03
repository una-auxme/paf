# Class: Collision Check

**Summary:** This document states the implemenation of th Collision Check class.

---

## Author

Julius Miller

## Date

03.12.2023

## Overview

Calculates if the ego vehicle will collide with the obstacle in front, assuming both travel with constant speed.

Improvements: Add local path into calculation

Input:

- ego_vehicle_speed (m/s)
- obstacle_speed (m/s)
- obstacle_distance (m)

Output:

- time to collision (s)
- meters to collision (m)
- emergency brake

Test: Ego travels with 50 km/h

Obstacle_1
Ego reaches obstacle after 8.49 seconds.
Ego reaches obstacle after 117.92 meters.

Safe Distance PAF 22: 28.78
Safe Distance Thumb: 38.89

Obstacle_2
Ego slower then car in front

Obstacle_3
Ego reaches obstacle after 10.11 seconds.
Ego reaches obstacle after 140.45 meters.

Safe Distance PAF 22: 28.78
Safe Distance Thumb: 38.89

Obstacle_4
Emergency Brake needed, 26.39
Ego reaches obstacle after 1.44 seconds.
Ego reaches obstacle after 20.00 meters.

Safe Distance PAF 22: 28.78
Safe Distance Thumb: 38.89
