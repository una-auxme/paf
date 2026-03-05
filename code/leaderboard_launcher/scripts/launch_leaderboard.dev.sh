#!/bin/bash
set -e

exec /workspace/code/leaderboard_launcher/scripts/launch_leaderboard.sh \
    --agent="/workspace/code/leaderboard_launcher/leaderboard_launcher/agent_dev.py" \
   --routes="/workspace/code/routes/routes_construction_sign.xml" \
 # --routes="${INTERNAL_WORKSPACE_DIR}/leaderboard/data/routes_devtest.xml" \
  --routes-subset=0


# routes-subset sets the index of the route to run
# Unset it to run all routes
# Look into ${INTERNAL_WORKSPACE_DIR}/leaderboard/leaderboard/leaderboard_evaluator.py for more parameters
