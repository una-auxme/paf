# Radar Raw Debugger

## Overview

The **Radar Raw Debugger** is a development tool used to inspect raw radar data directly from the simulation.

It subscribes to radar point cloud topics and prints detailed information about the incoming data. This helps developers understand the structure of the radar messages and verify correct sensor behavior.

The tool is intended for debugging and development purposes only and is not used in the runtime perception pipeline.

## Purpose

Radar data is provided as `sensor_msgs/PointCloud2`, which can be difficult to interpret without additional tools.

The Radar Raw Debugger helps to:

- Inspect the structure of radar point cloud messages  
- Verify that radar sensors are publishing correct data  
- Analyze available fields such as position and velocity  
- Debug issues in radar-based perception or clustering  

## Features

- Prints sample radar points for inspection  

- Works directly on raw ROS topics without preprocessing  

## Input

| Topic                  | Type                      | Description                     |
|------------------------|---------------------------|---------------------------------|
| `/carla/hero/RADAR0`   | `sensor_msgs/PointCloud2` | Raw radar data (front sensor)   |
| `/carla/hero/RADAR1`   | `sensor_msgs/PointCloud2` | Raw radar data (rear sensor)    |

## Usage

Run the debugger using:

```bash
ros2 run perception radar_raw_debugger
