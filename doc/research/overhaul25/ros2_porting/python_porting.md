# Title of wiki page

**Summary:** Overview of how the ROS1 python packages/nodes should be ported to ROS2 and what problems might occur

- [Generate Table of Contents](#generate-table-of-contents)
- [Some Content](#some-content)
- [more Content](#more-content)
- [Sources](#sources)

## Other porting options

### ros1_bridge

<https://github.com/ros2/ros1_bridge>
<https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html>

TODO

### Carla ROS Compatiblity Node

<https://github.com/carla-simulator/ros-bridge/blob/master/docs/ros_compatibility.md>

TODO

### Conclusion

TODO

## Package changes

<https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html>

This lists changes specifically for packages containing python nodes.
For general package changes, see [here](./README.md#migrate-package-base-files-to-colcon)

## Python file changes

1. Migrate packages
   - General
     - package python requirements (requirements.txt) may need major version upgrades -> API changes?
     - According to the [documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node),
       the directory structure of the packages will have to be adjusted.
     - Python packages need to have a proper setup.py
     - All nodes have to use the rclpy API instead of rospy
   - mapping_common
     - installation procedure will have to be updated since ROS 2 handles python dependencies differently
     - ROS 2 now natively supports py_test -> update test integration

## Necessary dependency upgrades

- behavior tree
  - the currently used py_trees version (0.7.6) is very old. The new version contains major API changes

## Version management options

<https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html>

### rosdep dependency management

### pip venv

### conda

The [documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html) states:  
*`make sure to use the system interpreter. If you use something like conda, it is very likely that the interpreter will not match the system interpreter and will be incompatible with ROS 2 binaries.`*

So package managers shipping their own python version are definitely not recommended.

### [uv](https://github.com/astral-sh/uv)

### Conclusion/Status
