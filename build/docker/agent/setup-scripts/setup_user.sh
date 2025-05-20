#!/bin/bash
set -e

echo "source ${INTERNAL_VENV}/bin/activate" >>~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >>~/.bashrc
echo "source ${ROOT_ROS_WS}/install/local_setup.bash" >>~/.bashrc
