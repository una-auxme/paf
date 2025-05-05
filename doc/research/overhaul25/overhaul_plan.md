# Overhaul plan

**Summary:** This page outlines possible steps for the overhaul/rework of the project during SoSe-25. (Projektmodul Peter Viechter, Sommersemester 2025)

- [Organisation](#organisation)
- [Dependency updates](#dependency-updates)
  - [Port to ROS 2](#port-to-ros-2)
- [Workflow improvements](#workflow-improvements)
  - [Use a singular Dockerfile with multiple build stages instead of the include](#use-a-singular-dockerfile-with-multiple-build-stages-instead-of-the-include)
  - [Remove user-specific instructions from the Dockerfile](#remove-user-specific-instructions-from-the-dockerfile)
  - [Retain user configurations across container recreations](#retain-user-configurations-across-container-recreations)
  - [Make it possible to keep Vs Code attached to the leaderboard container](#make-it-possible-to-keep-vs-code-attached-to-the-leaderboard-container)
  - [Make it possible to restart the leaderboard independently of Carla](#make-it-possible-to-restart-the-leaderboard-independently-of-carla)
  - [Fix Carla crashes](#fix-carla-crashes)
    - [Solutions](#solutions)
  - [Properly wait for container startup (Carla)](#properly-wait-for-container-startup-carla)
  - [Enable caching for pip and apt in the Dockerfile](#enable-caching-for-pip-and-apt-in-the-dockerfile)
  - [Add support for AMD GPUs](#add-support-for-amd-gpus)
- [General improvements](#general-improvements)
  - [Integration of unit-tests into the CI](#integration-of-unit-tests-into-the-ci)
  - [\[Feature\]: Implement Correct Synchronization Mechanism for CARLA Simulation Steps #701](#feature-implement-correct-synchronization-mechanism-for-carla-simulation-steps-701)
  - [Add more cython annotations in the mapping\_common package](#add-more-cython-annotations-in-the-mapping_common-package)

## Organisation

- Milestone for the overhaul?
- How to handle PRs?
  The ROS2 upgrade will be one giant PR spanning at least half of the Projektmodul. Splitting the PR makes little sense because a partial upgrade will not run.
  - It might be possible to create a second protected branch like *ros2-dev* and then make smaller PRs onto it (with limited ci checks)
- How to handle issues? Do sub-issues for the individual steps of the upgrade make sense?

## Dependency updates

The main goal of this overhaul is to update the dependencies since some of them are soon EOL.

### Port to ROS 2

The project currently uses ROS 1 noetic as its backbone. It is EOL in May 2025.

Required steps:

1. Build a basic ROS 2 Dockerfile.
   - This should make sure that the issues described [here](https://github.com/una-auxme/paf/issues/253) are fixable.
     - The carla<->[ros_bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/) officially only supports ROS 2 Foxy (already EOL)
     - Users were apparently able to get the bridge working with ROS 2 Humble: <https://github.com/carla-simulator/ros-bridge/issues/737>
       But it requires patched build files.  
   - The preferred ROS 2 Distribution is [Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) which is supported until 2029
   - The base image should be Ubuntu 24.04 based. To simplify GPU dependencies a GPU-vendor image might be used (Nvidia-CUDA/AMD-ROCM)
   - Ubuntu 24.04 ships with Python 3.12 (Upgrade from 3.8). Some dependencies might need updates. (requirements.txt)
   - Should the python dependencies use a venv to decouple from the fixed ubuntu versions?
   - The dependency versions of the leaderboard, scenario runner and bridge cannot be used with Python 3.12 and need to be updated. This might lead to errors.
     But the current setup also does not use these exact versions...
2. Make sure the leaderboard is able to start, send sensor data and control the simulation.
   - A dummy agent.py might be used for initial testing
3. Make sure all ROS package dependencies are met / replacements are found
4. Migrate package base files to colcon
   - package.xml
   - CMakeLists.txt
   - launch files
5. Migrate packages
   - General
     - package python requirements (requirements.txt) may need major version upgrades -> API changes?
     - According to the [documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node),
       the directory structure of the packages will have to be adjusted.
     - Python packages need to have a proper setup.py
     - All nodes have to use the rclpy API instead of rospy
   - mapping_common
     - installation procedure will have to be updated since ROS 2 handles python dependencies differently
     - ROS 2 now natively supports py_test -> update test integration
   - behavior tree
     - the currently used py_trees version (0.7.6) is very old. The new version contains major API changes

## Workflow improvements

Improvements that make working on the project easier.

These points mainly relate to the Docker configuration.

More ideas and experiments can be found in [[Feature]: Development container refactor #545](https://github.com/una-auxme/paf/issues/545)

### Use a singular Dockerfile with multiple build stages instead of the include

Reduced build time and does not require custom syntax (just buildkit, now enabled by default in docker)

### Remove user-specific instructions from the Dockerfile

### Retain user configurations across container recreations

Mainly keep the rqt config

### Make it possible to keep Vs Code attached to the leaderboard container

### Make it possible to restart the leaderboard independently of Carla

Will reduce simulation startup time

### Fix Carla crashes

The U4 Linux version of the Carla leaderboard always crashes after some time (vulkan memory leak?).

This is a problem when:

- running longer evaluations.
- Leaving carla running independently of the leaderboard

There is a fix, but no new leaderboard-specific version was released:

- <https://github.com/carla-simulator/carla/issues/7156>
- <https://github.com/carla-simulator/carla/issues/8703>

#### Solutions

- There are nightly builds available: <https://github.com/carla-simulator/carla/tree/ue4-dev?tab=readme-ov-file#download-carla>
  - It is unknown if they are up-to-date and stable enough for usage
- It might be possible to compile the ue4-dev branch from source... but this requires a lot of effort

### Properly wait for container startup (Carla)

Will reduce simulation startup time by not waiting an arbitrary amount of time

### Enable caching for pip and apt in the Dockerfile

### Add support for AMD GPUs

Already proven to work, will just use a different base docker image and compose config

## General improvements

General improvements to the projects that can be worked on if there is still time left.

### Integration of unit-tests into the CI

- [Improve the actions and their test coverage #385](https://github.com/una-auxme/paf/issues/385)

### [[Feature]: Implement Correct Synchronization Mechanism for CARLA Simulation Steps #701](https://github.com/una-auxme/paf/issues/701)

There might be a way to properly control the data-flow inside the projekt -> possible removal of the loop_sleep_time

Benefits:

- Reduced dead-time in simulation (less waiting time between steps)
- The simulation speed dynamically adjusts to the hosts CPU speed.
  - Currently, a host faster than loop_sleep_time will have dead-time
  - A host slower than loop_sleep_time will not leave enough time for computations inside the nodes. Datasets then start lagging (e.g. the behavior tree will use an outdated map for its decisions)

As long as the loop_sleep_time is set high enough for the given host-CPU, **the only real benefit is a faster simulation speed**

### Add more cython annotations in the mapping_common package

Performance improvements
