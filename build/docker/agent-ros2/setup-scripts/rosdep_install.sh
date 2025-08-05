#!/bin/bash
set -e

SRC_DIR=$1

rosdep update

rosdep install --ignore-src --from-paths "${SRC_DIR}" --rosdistro "${ROS_DISTRO}" -y
