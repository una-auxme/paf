#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
cd "${SCRIPT_DIR}"

_term() {
  echo "Caught SIGTERM signal!"
  kill -TERM "$(jobs -p)" 2>/dev/null
}
trap _term SIGTERM SIGINT

# Pull in the development bashrc
# It does not get automatically loaded, because the entrypoint is not an interactive shell
source "${INTERNAL_WORKSPACE_DIR}/dev.bashrc"

devbuild || echo "WARNING: Build failed, proceeding anyway..."

echo "source ${PAF_ROS_WS}/install/local_setup.bash" >>"${INTERNAL_WORKSPACE_DIR}/env.bash"
devsource || "WARNING: ROS workspace could not be sourced"

# Restore the home directory
sudo mkdir -p /home
sudo cp -ar --update=older /opt/home-bk/. -t /home/

ros2 run rqt_console rqt_console &
ros2 run rqt_gui rqt_gui &
ros2 run rviz2 rviz2 -d /workspace/rviz2.rviz &

"$@" &
wait "$!"
