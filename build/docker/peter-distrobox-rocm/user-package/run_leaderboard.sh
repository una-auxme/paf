#!/bin/bash
set -e

kill_leaderboard() {
  set +e
  pkill -f -SIGKILL /opt/leaderboard/leaderboard/leaderboard_evaluator.py
  pkill -f -SIGKILL CarlaUE4-Linux-Shipping
  rosnode kill -a
  pkill -f -SIGTERM run_roscore
  sleep 2
  pkill -f -SIGKILL run_roscore
  exit 0
}

trap kill_leaderboard EXIT INT TERM

cd ~
rm -rf ~/log/ros

cd "/${INTERNAL_WORKSPACE_DIR}/catkin_ws"
catkin_make

"${USER_PACKAGE_DIR}/run_roscore.sh" &

"${USER_PACKAGE_DIR}/run_carla.sh" &

python3 "${USER_PACKAGE_DIR}/wait_for_carla.py"

rqt_console &

ROUTES=${LEADERBOARD_ROOT}/data/routes_devtest.xml
DEBUG_CHALLENGE=0
AGENT=${WORKSPACE_DIR}/code/agent/src/agent/agent.py
CHECKPOINT_ENDPOINT=${WORKSPACE_DIR}/code/simulation_results.json
CHALLENGE_TRACK=MAP
CARLA_SIM_HOST=localhost
CARLA_SIM_WAIT_SECS=300

python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
  --debug=${DEBUG_CHALLENGE} \
  --checkpoint=${CHECKPOINT_ENDPOINT} \
  --track=${CHALLENGE_TRACK} \
  --agent=${AGENT} \
  --routes=${ROUTES} \
  --host=${CARLA_SIM_HOST} \
  --port=${CARLA_PORT} \
  --timeout=${CARLA_SIM_WAIT_SECS} &

trap kill_leaderboard EXIT INT TERM

while [ True ]; do
  sleep 1
done
