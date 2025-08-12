"""The mapping_common library is the core component of the **Intermediate Layer**

It contains the main dataclasses like Map end Entity
and all algorithms for working with them.

For a general overview of the **Intermediate Layer**
[look here](/doc/mapping/README.md).

### ROS interoperability

For easier programming, mapping_common does not use ROS message types directly,
but implements its own classes.

All classes with equivalent ROS message types can be converted into and from
ROS messages with the `<Object>.to_ros_msg()` and `<Class>.from_ros_msg(msg)`
methods.

### Cython

This module is compiled with [Cython](https://cython.readthedocs.io/en/latest/).

If changes have been made to this package,
catkin_make needs to be executed to apply them!
This step is automatically executed when using the
[docker-compose.leaderboard.yaml](/build/docker-compose.leaderboard.yaml).

"""

from typing import Optional
from rclpy.impl.rcutils_logger import RcutilsLogger
import rclpy.logging
import cython

if cython.compiled:
    print("mapping_common is compiled!")
else:
    print("mapping_common is not compiled!")

_mapping_common_logger: Optional[RcutilsLogger] = None


def set_logger(logger: RcutilsLogger):
    global _mapping_common_logger
    _mapping_common_logger = logger


def get_logger():
    if _mapping_common_logger is None:
        return rclpy.logging.get_logger("mapping_common")
    else:
        return _mapping_common_logger
