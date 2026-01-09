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

python3 "/workspace/code/leaderboard_launcher/leaderboard_launcher/wait_for_carla.py"

# Start leaderboard with arguments # edit "--routes" if you want a different testroute 
exec python3 /workspace/code/test/run_test.py \
  --host="${CARLA_SIM_HOST}" \
  --debug=0 \
  --routes="/workspace/code/routes/test.xml" \
  --agent="/workspace/code/leaderboard_launcher/leaderboard_launcher/agent_deploy.py" \
  --track=MAP \
  "${@}"
