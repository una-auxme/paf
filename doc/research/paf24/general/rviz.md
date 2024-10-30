# Research about RViz

**Summary:** This page contains information on how to use RViz and how it is integrated into the project.

- [General overview](#general-overview)
- [Displays panel](#displays-panel)
  - [Display types](#display-types)
    - [Camera](#camera)
    - [Image](#image)
    - [PointCloud(2)](#pointcloud2)
    - [Path](#path)
- [RViz configuration](#rviz-configuration)
- [Sources](#sources)

## General overview

Description from the git repository: **"rviz is a 3D visualizer for the Robot Operating System (ROS) framework."**

It can be used to visualize the state of the car in real-time.

- The Visualizer always has a 3D View panel in the middle. This is where all 3D data, from for example lidar and radar sensors, is shown.
- The most important panel is *Displays*. It is used to configure what data is displayed.
- All other panels are available under *Panels* in the menu bar.
- Panels can be regrouped by dragging their title bar.

## Displays panel

The *Displays* panel contains a list all the currently visualized data-displays and allows changing their settings and visibility.

The default configuration currently displays the center camera, lidar and radar point clouds, planned path and segmentation image.

Individual data-displays can be added and removed in the lower menu bar of the panel. Adding works in two ways:

- By type: The user then has to manually set up the settings including the topic which the display visualizes.
- By topic: In the *By topic* tab, RViz lists all available ros topics and allows to easily add them for visualization.

Do not forget to give the data-display a proper name when adding it (Renaming with F2 is also possible).

### Display types

There are several display types that can be added.
Depending on the type there are different settings available for the display.
The *Topic* setting controls from which ros topic the displays gets its data.

The most important display types are:

#### Camera

Shows an image from a camera. Allows overlaying other data on top of the image including *Path* and *PointCloud2* (*Visibility* setting)

Adding a camera also adds a new panel with the camera image.

#### Image

Shows an image. Also works for camera topics.

Adding an image also adds a new panel with the image.

#### PointCloud(2)

Shows a point cloud in the 3D View

#### Path

Shows a path in the 3D View

## RViz configuration

RViz can be fully configured with the GUI. The settings may then be saved with *File->Save Config*.

The default configuration file is located at [code/agent/config/rviz_config.rviz](../../../../code/agent/config/rviz_config.rviz)
and this path is defined in [code/agent/launch/agent.launch](../../../../code/agent/launch/agent.launch). It can be changed to use a different default config when running the leaderboard.

## Sources

<https://github.com/ros-visualization/rviz>

<http://wiki.ros.org/rviz/UserGuide>
