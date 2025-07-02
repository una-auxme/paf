#!/bin/bash
set -e

_term() {
  echo "Caught SIGTERM signal!"
  kill -TERM "$(jobs -p)" 2>/dev/null
}
trap _term SIGTERM SIGINT

# Required otherwise the leaderboard does not reliably print to the console when started from a shell script
export PYTHONUNBUFFERED=true

source "${INTERNAL_WORKSPACE_DIR}/env.bash"

python3 /wait_for_carla.py

"$@" &
wait "$!"
