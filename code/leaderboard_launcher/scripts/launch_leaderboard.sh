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

checkpoint_path="./simulation_results.json"
for ((i = 1; i <= $#; ++i)); do
  arg="${!i}"
  if [[ "${arg}" == "--checkpoint" ]]; then
    next_index=$((i + 1))
    checkpoint_path="${!next_index}"
  elif [[ "${arg}" == --checkpoint=* ]]; then
    checkpoint_path="${arg#--checkpoint=}"
  fi
done

python3 - <<'PY'
import sys

sys.path.insert(0, "/workspace/code/paf_common")

from paf_common.route_metrics import reset_route_metrics_file

reset_route_metrics_file()
PY

python3 "/workspace/code/leaderboard_launcher/leaderboard_launcher/wait_for_carla.py"

# Start leaderboard with arguments
set +e
python3 "${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py" \
  --host="${CARLA_SIM_HOST}" \
  --track=MAP \
  "${@}"
exit_code=$?
set -e

python3 - <<PY
import sys

sys.path.insert(0, "/workspace/code/paf_common")

from paf_common.route_metrics import merge_route_metrics_into_checkpoint

metrics = merge_route_metrics_into_checkpoint("${checkpoint_path}")
if metrics.get("metrics"):
    print("PAF ROUTE METRICS")
    for metric_name, metric_value in sorted(metrics["metrics"].items()):
        print(f"{metric_name}: {metric_value}")
PY

exit "${exit_code}"
