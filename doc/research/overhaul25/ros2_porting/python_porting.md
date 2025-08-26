# Python porting

**Summary:** Overview of how the ROS1 python packages/nodes should be ported to ROS2 and what problems might occur

- [Package changes](#package-changes)
  - [setup.py](#setuppy)
- [Python file changes](#python-file-changes)
  - [General](#general)
  - [mapping\_common](#mapping_common)
  - [debugging](#debugging)
- [Necessary dependency upgrades](#necessary-dependency-upgrades)
- [Dependency management options](#dependency-management-options)
  - [rosdep dependency management](#rosdep-dependency-management)
  - [pip ~~(optional: venv)~~](#pip-optional-venv)
  - [conda](#conda)
  - [uv](#uv)
  - [Conclusion/Status](#conclusionstatus)
- [Other porting options](#other-porting-options)
  - [ros1\_bridge](#ros1_bridge)
  - [Carla ROS Compatiblity Node](#carla-ros-compatiblity-node)
  - [Conclusion](#conclusion)

## Package changes

- <https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html>

This chapter contains required changes specifically for ROS packages containing python nodes.
For general package changes, see [here](./README.md#migrate-package-base-files-to-colcon)

### setup.py

This file now has to properly handle the installation of python packages/nodes. (Optionally also their dependencies)

**These will have to be fully rewritten.** (Including adjustments to the src folder structure)

According to the [documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node), the python packages are not put into the src folder anymore.  
It should still be possible to leave them in src, but it's not the default.

## Python file changes

This chapter contains required changes for the python nodes/files.

### General

- Package python requirements (requirements.txt) may need major version upgrades -> API changes?
- All nodes have to use the **rclpy API instead of rospy**
  - The nodes now need to be based on a class that inherits from `rclpy.node.Node` instead of using global functions.
  - This [example](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node) should translate nicely into the nodes that just subscribe to and publish topics.
  - Setting up timers, publisher, subscribers and services is quite similar to rospy and the structure of existing nodes does not have to change much.
  - **The main problem is calling blocking functions (services) inside a callback (timer, subscription, etc..) which leads to deadlocks with the default single-threaded executor.** [Source](https://docs.ros.org/en/jazzy/How-To-Guides/Sync-Vs-Async.html). Methods for handling:
    - **The recommended method is to use async calls to services.** But this requires the developer to query any futures properly.
    - Multithreading is another option, but it requires careful handling of shared resources. For this the `MultiThreadedExecutor` can be used.
    - Pretty good guide? <https://nu-msr.github.io/ros_notes/ros2/spin.html>
    - Callbacks in ROS support both functions and coroutines (async "functions")
    - If a callback is a coroutine, it will be automatically queried by the executor (and the developer does not need to care about it)  
      **➡ defining all callbacks as coroutines might be a good practice? (at least the ones that use service calls).**
    - Manual handling of CallbackGroups might still be necessary for proper execution -> further research required.
      - <https://github.com/m-elwin/async_experiments/blob/main/async_experiments/async_client.py>
  - **➡ No major problems expected, but guidelines for usage of async or multithreading have to be worked out and properly documented.** (Experiments required first)

### mapping_common

- installation procedure will have to be updated since ROS 2 handles python dependencies differently
- ROS 2 now natively supports py_test -> update test integration

### debugging

The integration of the debugger will have to be updated sind python node are integrated/started differently in ROS2.

TODO: Figure out a solution for easy integration into the launch config.

This will be relevant AFTER the main porting effort has been successful.

## Necessary dependency upgrades

- behavior tree
  - the currently used py_trees version (0.7.6) is very old. The new version contains major API changes

## Dependency management options

- <https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html>

Two questions:

- Where to save the list of required dependencies?
- What package manager to use?

### rosdep dependency management

Use package.xml + rosdep for dependency management

Pros:

- Official ROS dependency manager
- Per package dependencies (Clearer what needs what)

Cons:

- Per package dependencies (Might lead to conflicts)
- Requires each python dependency to have a rosdep key
- Versioning seems inflexible if very up-to-date packages are needed

### pip ~~(optional: venv)~~

The required dependencies can be either tracked in a project-level `requirements.txt` or in each of the package's setup.py.

Pros:

- The [documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html) directly references this installation method as a possible solution.
- Pip is the python ecosystem standard
- ~~[Use a venv for python dependencies](../improvements/docker.md#use-a-venv-for-python-dependencies)~~

Cons:

- None? maybe performance, but that is just relevant for docker build times when requirements changed (so not really)

### conda

The [documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html) states:  
*`make sure to use the system interpreter. If you use something like conda, it is very likely that the interpreter will not match the system interpreter and will be incompatible with ROS 2 binaries.`*

So package managers shipping their own python version are definitely not recommended.

### [uv](https://github.com/astral-sh/uv)

Pros:

- Package installation speed
- Better dependency handling
- Pip compatible interface

Cons:

- Nowhere mentioned in the ROS documentation
- Installation speed does only matter for docker build times and only when requirements changed
- [Caching the PIP_CACHE_DIR](../improvements/docker.md#enable-caching-for-pip-and-apt-in-the-dockerfile) will already provide a substantial speedup by reducing download times.

### Conclusion/Status

Recommendation: Use pip + requirements.txt (no venv) as this combination is well-documented, widely-used, and officially compatible with ROS 2.

## Other porting options

This chapter describes other options to port ROS1 python packages/nodes to ROS2. (Will NOT be used)

### ros1_bridge

- <https://github.com/ros2/ros1_bridge>  
- <https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html>

"This package provides a network bridge which enables the exchange of messages between ROS 1 and ROS 2."

Pros:

- Existing ROS1 nodes can be used with ROS2 without porting.

Cons:

- Increased complexity + dependencies
- Limitations like: "support is limited to only the message/service types available at compile time of the bridge"
- **Only supported until ROS2 Humble -> EOL 2027** -> Only a delay of the porting problem, not a solution.

### Carla ROS Compatiblity Node

- <https://github.com/carla-simulator/ros-bridge/blob/master/docs/ros_compatibility.md>

The ros_bridge package from Carla provides a ROS Compatiblity Node that works with both ROS1 and ROS2.

This node is already partially used in the project. Wider adoption in the project never happened, because the API is very limited.

Pros:

- Nodes work "natively" with ROS1 and ROS2

Cons:

- Very limited API
- Barely maintained just like the ros_bridge itself (last update 2021)

### Conclusion

Both options seem to only delay the porting problem, not solve it.
Compatibility between ROS1 and ROS2 will not be needed in the future, so it makes sense to properly and fully port the project to ROS2 now.
