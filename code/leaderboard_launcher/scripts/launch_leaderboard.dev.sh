#!/bin/bash
set -e

exec /workspace/code/leaderboard_launcher/scripts/launch_leaderboard.sh \
  --routes="${INTERNAL_WORKSPACE_DIR}/leaderboard/data/routes_devtest.xml" \
  --routes-subset=0

# --routes="/workspace/code/routes/routes_highway.xml" \

# routes-subset sets the index of the route to run
# Unset it to run all routes
# Look into ${INTERNAL_WORKSPACE_DIR}/leaderboard/leaderboard/leaderboard_evaluator.py for more parameters
