# Radar Node

**Summary:** This page explains what the radar sensor does.

- [Radar offsets](#radar-offsets)
- [Radar specification](#radar-specification)
- [Radar data output for each detected point](#radar-data-output-for-each-detected-point)
- [Todo](#todo)

## Radar offsets

- x: 2
- y: 0
- z: 0.7

## Radar specification

- points per second: 1500 points generated by all lasers per second
- maximum range: 100 meters
- horizontal fov: 30 degrees
- vertical fov: 30 degrees

## Radar data output for each detected point

- x
- y
- z
- Range
- Velocity
- AzimuthAngle
- ElevationAngle

## Todo

- Discuss further processing of radar data
- Combine lidar, radar and camera data