#!/bin/bash
set -e

cd "${INTERNAL_WORKSPACE_DIR}"

# Reset full ROS/Python environment
source "${INTERNAL_WORKSPACE_DIR}/scripts/reset_env.bash"
reset_env

# Source the leaderboard workspace environment (excludes AGENT_DEPS_ROS_WS)
source "${INTERNAL_WORKSPACE_DIR}/env.leaderboard.bash"

# Source leaderboard specific venv
source leaderboard_venv/bin/activate

python3 "/workspace/code/leaderboard_launcher/agent/wait_for_carla.py"

# Start leaderboard with arguments
exec python3 "${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py" \
  --host="${CARLA_SIM_HOST}" \
  --agent="/workspace/code/leaderboard_launcher/agent/agent.py" \
  "${@}"
