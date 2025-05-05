#!/bin/bash
set -e

SRC_DIR=$1

source "${INTERNAL_VENV}/bin/activate"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

apt-get update
rosdep update

rosdep install --ignore-src --from-paths "${SRC_DIR}" --rosdistro "${ROS_DISTRO}" -y
