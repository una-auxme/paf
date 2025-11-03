#!/bin/bash
set -e

exec /workspace/code/leaderboard_launcher/scripts/launch_leaderboard.sh \
  --agent="/workspace/code/leaderboard_launcher/leaderboard_launcher/agent_deploy.py" \
  --routes="${INTERNAL_WORKSPACE_DIR}/leaderboard/data/routes_devtest.xml"

# Look into launch_leaderboard.dev.sh for more info
