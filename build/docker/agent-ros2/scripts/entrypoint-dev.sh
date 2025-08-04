#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
cd "${SCRIPT_DIR}"

_term() {
  echo "Caught SIGTERM signal!"
  kill -TERM "$(jobs -p)" 2>/dev/null
}
trap _term SIGTERM SIGINT

# Delete local python packages if they exist
if [ -d ~/.local ]; then
  rm -r ~/.local
fi
# Restore the home directory
sudo mkdir -p /home
sudo cp -ar --update=older /opt/home-bk/. -t /home/

# Pull in the development bashrc
# It does not get automatically loaded, because the entrypoint is not an interactive shell
source "${INTERNAL_WORKSPACE_DIR}/dev.bashrc"

devbuild || echo "WARNING: Build failed, proceeding anyway..."

echo "source ${PAF_ROS_WS}/install/local_setup.bash" >>"${INTERNAL_WORKSPACE_DIR}/env.bash"
devsource || "WARNING: ROS workspace could not be sourced"

ros_gui_params=(--ros-args --param use_sim_time:=true)
ros2 run rqt_console rqt_console "${ros_gui_params[@]}" &
ros2 run rqt_gui rqt_gui "${ros_gui_params[@]}" &
ros2 run rviz2 rviz2 -d /workspace/rviz2.rviz "${ros_gui_params[@]}" &
code /workspace &

"$@" &
wait "$!"
