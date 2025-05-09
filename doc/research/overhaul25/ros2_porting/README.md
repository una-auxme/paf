# ROS2 porting

**Summary:** Required steps for porting the project to ROS 2

The project currently uses ROS 1 noetic as its backbone. It is EOL in May 2025.

- [Steps](#steps)
  - [1. Build a basic ROS 2 Dockerfile](#1-build-a-basic-ros-2-dockerfile)
    - [Steps (Dockerfile)](#steps-dockerfile)
    - [Problems (Dockerfile)](#problems-dockerfile)
    - [Status (Dockerfile)](#status-dockerfile)
  - [2. ROS package dependencies](#2-ros-package-dependencies)
  - [3. Take measurements with the ROS1 codebase](#3-take-measurements-with-the-ros1-codebase)
    - [Performance indicators](#performance-indicators)
  - [4. Main porting effort: Migrate ROS packages](#4-main-porting-effort-migrate-ros-packages)
    - [Migrate package base files to colcon](#migrate-package-base-files-to-colcon)
    - [Migrate Python packages/code to ROS2](#migrate-python-packagescode-to-ros2)
  - [5. Compare measurements to identify problems](#5-compare-measurements-to-identify-problems)
  - [6. Update documentation](#6-update-documentation)
- [CI plan](#ci-plan)
- [Risks and Mitigation](#risks-and-mitigation)

## Steps

### 1. Build a basic ROS 2 Dockerfile

This should make sure that the issues described [here](https://github.com/una-auxme/paf/issues/253) are fixable.

The preferred ROS 2 Distribution is [Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) which is supported until 2029.
The main reason for using jazzy is the longer support window than Humble Hawksbill (EOL 2027).
Should issues arise specifically related to jazzy, a fallback to humble is relatively easy by switching the ROS distribution in the Dockerfile.

The base docker image is Ubuntu 24.04 based. To simplify GPU dependencies a GPU-vendor image is used (Nvidia-CUDA/AMD-ROCM).
This removes a lot of complex steps from the Dockerfile. (And manual ROS installation is a lot easier)

Further improvements the docker/build system can be found [here](../improvements/docker.md).
Some of them will be implemented directly as part of the ROS2 porting effort.

#### Steps (Dockerfile)

1. [DONE] Create a Dockerfile including the leaderboard, scenario runner and bridge and successfully build it
2. [KINDA DONE, NEEDS VALIDATION] Install the required python dependencies
3. Make sure the leaderboard is able to start, send sensor data and control the simulation
   - A dummy agent.py might be used for initial testing
  
#### Problems (Dockerfile)

- [FIXED] The carla<->[ros_bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/) officially only supports ROS 2 Foxy (already EOL)  
  Users were apparently able to get the bridge working with ROS 2 Humble: <https://github.com/carla-simulator/ros-bridge/issues/737>  
  But it requires patched build files.
- Ubuntu 24.04 ships with Python 3.12 (Upgrade from 3.8). Some dependencies have to be upgraded. More info below and [here](./python_porting.md#necessary-dependency-upgrades)
- The locked python dependencies of the leaderboard, scenario runner and bridge cannot be used with Python 3.12 and need to be updated.
  - This may lead to unforeseen errors.
  - The current setup also does not use these exact versions...
  - Affected packages:
    - TODO

#### Status (Dockerfile)

TODO

### 2. ROS package dependencies

Make sure all ROS package dependencies are met / replacements are found

TODO: List changed or deprecated packages

### 3. Take measurements with the ROS1 codebase

TODO

#### Performance indicators

TODO

### 4. Main porting effort: Migrate ROS packages

<https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html>

#### Migrate package base files to colcon

TODO: Check format differences

- package.xml
- CMakeLists.txt
- launch files

#### Migrate Python packages/code to ROS2

The planned process for porting individual python packages and nodes is described [here](./python_porting.md).

### 5. Compare measurements to identify problems

### 6. Update documentation

This is a rough overview of necessary documentation updates if the ROS2 port is successful

- Node creation template
- Introduction to rclpy? Remove all references to rospy.
- docker-compose usage&build instructions
- setup instructions and scripts
- Update any broken links inside the existing docs

## CI plan

TODO

## Risks and Mitigation

TODO
