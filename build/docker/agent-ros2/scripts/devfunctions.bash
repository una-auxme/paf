#!/bin/bash
# This script contains development helper functions

# This message prints on every interactive bash shell that opens
cat <<EOF
Development utility functions:
- devsource: sources the ${INTERNAL_WORKSPACE_DIR}/env.bash (includes the ${PAF_ROS_WS}/install/local_setup.bash)
- devbuild: colcon builds the ${PAF_ROS_WS} (colcon build --symlink-install)
- pytrees.viewer: Launches the py-trees tree viewer for behavior trees
                   Note: You need to have a behavior tree running (like the agent) for this to show anything
- leaderboard.dev: Starts code/leaderboard_launcher/scripts/launch_leaderboard.dev.sh
                   You can adjust leaderboard parameters (like which route to run) there.
                   The leaderboard then automatically launches code/agent/launch/agent.dev.persistent.xml
                   Quitting: Ctrl+c does not work on the leaderboard. Use Right-Click->Kill Terminal
- agent.dev: Launches the agent (ros2 launch agent agent.dev.xml); Ctrl+c to stop the agent
EOF

# This function sources the ROS /workspace
devsource() {
  source "${INTERNAL_WORKSPACE_DIR}/env.bash"
  echo "${INTERNAL_WORKSPACE_DIR}/env.bash sourced."
}
export -f devsource

# This function builds the ROS /workspace and sources it again afterwards
devbuild() {
  (
  cd "${PAF_ROS_WS}" || return $?

  # Build the ros workspace
  colcon build --symlink-install --continue-on-error || return $?
  )

  cat <<EOF

ROS workspace built successfully.
Do not forget to source the ros workspace again if you changed anything in the package.xml or setup.py files.
This shell window will do it automatically. In other open shells run: devsource
EOF

  devsource || return $?
}
export -f devbuild

leaderboard.dev() {
  /workspace/code/leaderboard_launcher/scripts/launch_leaderboard.dev.sh
}
export -f leaderboard.dev

agent.dev() {
  (
  cd "${INTERNAL_WORKSPACE_DIR}" || return $? # This makes sure any downloads (yolo) land in INTERNAL_WORKSPACE_DIR
  ros2 launch agent agent.dev.xml
  )
}
export -f agent.dev

pytrees.viewer() {
  (
  py-trees-tree-viewer --tree /behavior_agent/tree
  )
}
export -f pytrees.viewer
