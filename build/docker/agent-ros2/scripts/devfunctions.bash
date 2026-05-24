#!/bin/bash
# This script contains development helper functions

# This message prints on every interactive bash shell that opens
cat <<EOF
Development utility functions:
- devsource: sources the ${INTERNAL_WORKSPACE_DIR}/env.bash (includes the ${PAF_ROS_WS}/install/local_setup.bash)
- devbuild: colcon builds the ${PAF_ROS_WS} (colcon build --symlink-install)
- devbuild.pkg <name>: colcon builds only selected package for faster local iteration
- pytrees.viewer: Launches the py-trees tree viewer for behavior trees
                   Note: You need to have a behavior tree running (like the agent) for this to show anything
- leaderboard.dev: Starts code/leaderboard_launcher/scripts/launch_leaderboard.dev.sh
                   You can adjust leaderboard parameters (like which route to run) there.
                   The leaderboard then automatically launches code/agent/launch/agent.dev.persistent.xml
                   Quitting: Ctrl+c does not work on the leaderboard. Use Right-Click->Kill Terminal
- agent.dev: Launches the agent (ros2 launch agent agent.dev.xml); Ctrl+c to stop the agent
- leaderboard.test: Launches the Simulation test at /code/leaderboard_launcher/scripts/launch_leaderboard.test.sh
                    Cleans up stale run_test.py process groups first unless PAF_LEADERBOARD_TEST_SKIP_CLEANUP=1
                    Override the traffic manager port with PAF_TRAFFIC_MANAGER_PORT=<port>
                    Kill Terminal to end
- ruff.lint: Manually trigger the ruff linter to check the python files
- ruff.fix-lint: Apply the safe fixes that the ruff linter encounters during linting
- ruff.check-format: Manually trigger the ruff formatter to check if the formatting of the python files is correct
- ruff.format: Apply the ruff formatting to the python files
- dep.check: Validates rosdep + pip dependency health for the current container environment
- dep.sync: Installs rosdep and pip dependencies from the repository manifests
EOF

# This function sources the ROS /workspace
devsource() {
  source "${INTERNAL_WORKSPACE_DIR}/env.bash"

  if [ -f "${PAF_ROS_WS}/install/local_setup.bash" ]; then
    source "${PAF_ROS_WS}/install/local_setup.bash"
  fi

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

devbuild.pkg() {
  if [ -z "${1:-}" ]; then
    echo "Usage: devbuild.pkg <package_name>"
    return 2
  fi

  (
  cd "${PAF_ROS_WS}" || return $?

  colcon build --symlink-install --continue-on-error --packages-select "$1" || return $?
  )

  cat <<EOF

Package '$1' built successfully.
Source updates in other shells with: devsource
EOF

  devsource || return $?
}
export -f devbuild.pkg

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

leaderboard.test(){
  (
    if [[ "${PAF_LEADERBOARD_TEST_SKIP_CLEANUP:-0}" != "1" ]]; then
      leaderboard.test.cleanup
    fi

    /workspace/code/leaderboard_launcher/scripts/launch_leaderboard.test.sh "$@"
  )
}
export -f leaderboard.test

leaderboard.test.cleanup() {
  local pid
  local pgid
  local stale_pids

  stale_pids=$(pgrep -f '/workspace/code/test/run_test.py' || true)
  if [[ -z "$stale_pids" ]]; then
    return 0
  fi

  echo "Stopping stale leaderboard.test process groups..."
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    pgid=$(ps -o pgid= -p "$pid" | tr -d '[:space:]')
    [[ -z "$pgid" ]] && continue
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done <<< "$stale_pids"

  sleep 2

  stale_pids=$(pgrep -f '/workspace/code/test/run_test.py' || true)
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    pgid=$(ps -o pgid= -p "$pid" | tr -d '[:space:]')
    [[ -z "$pgid" ]] && continue
    kill -KILL -- "-$pgid" 2>/dev/null || true
  done <<< "$stale_pids"
}
export -f leaderboard.test.cleanup

ruff.lint() {
  (
    ruff check /workspace/code/
  )
}
export -f ruff.lint

ruff.fix-lint() {
  (
    ruff check /workspace/code/ --fix
  )
}
export -f ruff.fix-lint

ruff.check-format() {
  (
    ruff format /workspace/code/ --check
  )
}
export -f ruff.check-format

ruff.format() {
  (
    ruff format /workspace/code/
  )
}
export -f ruff.format

dep.check() {
  (
    "${INTERNAL_WORKSPACE_DIR}/scripts/dependency-sync.sh" check
  )
}
export -f dep.check

dep.sync() {
  (
    "${INTERNAL_WORKSPACE_DIR}/scripts/dependency-sync.sh" sync
  )
}
export -f dep.sync
