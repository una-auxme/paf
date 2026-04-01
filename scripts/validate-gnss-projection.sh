#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PAF_DEV_CONTAINER:-build-agent-dev-1}"
CARLA_HOST="${CARLA_SIM_HOST:-carla-simulator}"
ROLE_NAME="${PAF_ROLE_NAME:-hero}"
SIDE_NODE_NAME="${PAF_GNSS_SIDE_NODE_NAME:-gps_transform_projection_check}"
OUTPUT_TOPIC="${PAF_GNSS_OUTPUT_TOPIC:-/odometry/gps_projection_check}"
SAMPLE_COUNT="${PAF_GNSS_SAMPLE_COUNT:-10}"
TIMEOUT_SEC="${PAF_GNSS_TIMEOUT_SEC:-20}"
MEAN_THRESHOLD_M="${PAF_GNSS_MEAN_THRESHOLD_M:-}"
MAX_THRESHOLD_M="${PAF_GNSS_MAX_THRESHOLD_M:-}"

cleanup() {
  docker exec "${CONTAINER_NAME}" bash -lc \
    "pkill -f '${SIDE_NODE_NAME}|gps_transform --ros-args.*${SIDE_NODE_NAME}' || true" \
    >/dev/null 2>&1 || true
}

trap cleanup EXIT

if ! docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "Dev container '${CONTAINER_NAME}' is not running. Start the dev stack first." >&2
  exit 1
fi

if ! docker exec "${CONTAINER_NAME}" bash -lc \
  "pgrep -f '/workspace/code/test/run_test.py' >/dev/null"; then
  echo "No live leaderboard route detected in ${CONTAINER_NAME}. Start leaderboard.test first." >&2
  exit 1
fi

if ! docker exec "${CONTAINER_NAME}" bash -lc \
  "python3 -c 'import pyproj'" >/dev/null 2>&1; then
  echo "pyproj is missing in ${CONTAINER_NAME}; synchronizing dependencies..."
  docker exec "${CONTAINER_NAME}" bash -lc \
    "source /internal_workspace/dev.bashrc >/dev/null 2>&1 && bash /workspace/build/docker/agent-ros2/scripts/dependency-sync.sh sync"
fi

cleanup

docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /internal_workspace/dev.bashrc >/dev/null 2>&1
  export CARLA_SIM_HOST='${CARLA_HOST}'
  nohup ros2 run localization gps_transform \
    --ros-args \
    -r __node:='${SIDE_NODE_NAME}' \
    -p role_name:='${ROLE_NAME}' \
    -p position_use_ground_truth:=false \
    -p use_sim_time:=True \
    -r /odometry/gps:='${OUTPUT_TOPIC}' \
    >/tmp/${SIDE_NODE_NAME}.log 2>&1 &
"

docker exec "${CONTAINER_NAME}" bash -lc "
  export PAF_GNSS_SIDE_OUTPUT_TOPIC='${OUTPUT_TOPIC}'
  export PAF_GNSS_SAMPLE_COUNT='${SAMPLE_COUNT}'
  export PAF_GNSS_TIMEOUT_SEC='${TIMEOUT_SEC}'
  export PAF_GNSS_CARLA_HOST='${CARLA_HOST}'
  export PAF_GNSS_MEAN_THRESHOLD_M='${MEAN_THRESHOLD_M}'
  export PAF_GNSS_MAX_THRESHOLD_M='${MAX_THRESHOLD_M}'
  source /internal_workspace/dev.bashrc >/dev/null 2>&1
  python3 - <<'PY'
import json
import math
import os
import time

import carla
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Probe(Node):
    def __init__(self, world):
        super().__init__('gps_projection_probe')
        self.samples = []
        self.world = world
        self.create_subscription(
            Odometry,
            os.environ['PAF_GNSS_SIDE_OUTPUT_TOPIC'],
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == '${ROLE_NAME}':
                hero = actor
                break
        if hero is None:
            return

        location = hero.get_location()
        carla_enu = (location.x, -location.y, location.z)
        odom_xyz = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        self.samples.append(
            {
                'error_m': math.dist(odom_xyz, carla_enu),
                'projection_xyz': odom_xyz,
                'carla_xyz': carla_enu,
            }
        )


def maybe_threshold(name: str):
    value = os.environ.get(name)
    return None if value in (None, '') else float(value)


def connect_world(deadline: float):
    last_error = None
    while time.time() < deadline:
        try:
            client = carla.Client(os.environ['PAF_GNSS_CARLA_HOST'], 2000)
            client.set_timeout(10.0)
            return client.get_world()
        except Exception as exc:
            last_error = exc
            time.sleep(0.5)

    raise RuntimeError(last_error or 'Timed out while connecting to CARLA.')


def main():
    rclpy.init()
    deadline = time.time() + float(os.environ['PAF_GNSS_TIMEOUT_SEC'])
    probe = Probe(connect_world(deadline))
    target_samples = int(os.environ['PAF_GNSS_SAMPLE_COUNT'])

    while time.time() < deadline and len(probe.samples) < target_samples:
        rclpy.spin_once(probe, timeout_sec=0.5)

    if not probe.samples:
        print('No samples collected from the sidecar GNSS projection node.', flush=True)
        probe.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    errors = [sample['error_m'] for sample in probe.samples]
    result = {
        'sample_count': len(probe.samples),
        'gps_err_mean_m': sum(errors) / len(errors),
        'gps_err_max_m': max(errors),
        'last_projection_xyz': probe.samples[-1]['projection_xyz'],
        'last_carla_xyz': probe.samples[-1]['carla_xyz'],
    }
    print(json.dumps(result, indent=2), flush=True)

    mean_threshold = maybe_threshold('PAF_GNSS_MEAN_THRESHOLD_M')
    max_threshold = maybe_threshold('PAF_GNSS_MAX_THRESHOLD_M')
    if mean_threshold is not None and result['gps_err_mean_m'] > mean_threshold:
        probe.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)
    if max_threshold is not None and result['gps_err_max_m'] > max_threshold:
        probe.destroy_node()
        rclpy.shutdown()
        raise SystemExit(3)

    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PY"
