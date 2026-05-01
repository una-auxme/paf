Created: 2026-05-01T08:14:00+02:00
Last updated: 2026-05-01T08:31:00+02:00

# Build Validation Progress

## Motivation

Run the full local build path after the route-sync and issue-sweep changes, and
remove the Docker image build blocker observed in the GPU base stage.

## Current chat state

The local ROS workspace build passed inside `build-agent-dev-1` with
`devbuild`, completing all 12 project packages.

The CUDA dev image initially blocked in the GPU base stage while
`add-apt-repository -y ppa:kisak/turtle` waited on the Launchpad PPA. The
Dockerfile now keeps the Kisak PPA as the preferred source, but bounds that
step with `timeout 60s` and falls back to the Ubuntu Mesa/Vulkan packages when
the PPA is unavailable.

After that change, both Docker image builds completed:

- `agent-dev-luttkule`
- `agent-deploy-luttkule`

The CARLA route validation path was then exercised with the split development
startup flow:

1. start `carla-simulator`,
2. run `leaderboard.dev`,
3. run `agent.dev` once the route is active,
4. stop the agent, leaderboard evaluator, bridge, and simulator before handoff.

That run exposed a preplanner startup issue: the global route could be rejected
when the first route pose was behind the current ego pose. The preplanner now
aligns to the nearest route pose within the existing
`distance_spawn_to_first_wp` tolerance before trimming the route for trajectory
generation.

The behavior-agent lane decisions were also updated so an absent adjacent lane
is no longer treated the same as a free lane.

## Validation

- Green check:
  `docker exec build-agent-dev-1 bash -lc 'source /internal_workspace/dev.bashrc && devbuild'`
  passed with `Summary: 12 packages finished [17.0s]`.
- Green check:
  `docker compose --env-file build/.env -f build/docker-compose.dev.cuda.yml build agent-dev`
  built `agent-dev-luttkule`.
- Green check:
  `docker compose --env-file build/.env -f build/docker-compose.deploy.cuda.yml build agent-deploy`
  built `agent-deploy-luttkule`; its deploy-stage `colcon build` passed with
  `Summary: 12 packages finished [1min 16s]`.
- Green check:
  `docker exec build-agent-dev-1 bash -lc 'cd /workspace && source /internal_workspace/dev.bashrc && devbuild.pkg mapping'`
  rebuilt the mapping package after the lane-context helper change.
- Green check:
  `docker exec build-agent-dev-1 bash -lc 'cd /workspace && source /internal_workspace/dev.bashrc && devbuild.pkg planning'`
  rebuilt the planning package after behavior and global-planner changes.
- Green check:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest code/mapping/test/test_mapping_common/test_map.py -q`
  passed in `build-agent-dev-1` with `4 passed`.
- Green check:
  `uv run --with pytest==9.0.2 --with numpy python -m pytest code/test/test_planning_regression.py -q`
  passed with `7 passed`.
- Green check:
  `uv run --with ruff==0.14.8 ruff check` and
  `uv run --with ruff==0.14.8 ruff format --check` passed for the touched
  Python files.
- CARLA route smoke check:
  `leaderboard.dev` plus `agent.dev` reached route runtime, the global
  trajectory service became available, `MotionPlanning` consumed it, and
  `vehicle_controller` published timestamped `/carla/hero/vehicle_control_cmd`
  messages. The run was stopped before full route completion after validating
  startup, global trajectory delivery, and control-command publication.
- CARLA cleanup check:
  no `carla-simulator`, `CarlaUE4`, `leaderboard`, or `run_test.py` runtime
  processes were left running. The only matching Docker container name was the
  pre-existing BuildKit helper `buildx_buildkit_carla-builder0`, not a CARLA
  simulator.

## Remaining blockers

- The full leaderboard route was not left running to completion. The smoke run
  validated startup and command publication, but not route score or completion.
