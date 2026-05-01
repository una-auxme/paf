Created: 2026-05-01T08:14:00+02:00
Last updated: 2026-05-01T08:14:00+02:00

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

No CARLA simulator service was launched for this build pass.

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
- CARLA cleanup check:
  no `carla-simulator`, `CarlaUE4`, `leaderboard`, or `run_test.py` runtime
  processes were left running. The only matching Docker container name was the
  pre-existing BuildKit helper `buildx_buildkit_carla-builder0`, not a CARLA
  simulator.

## Remaining blockers

- CARLA route validation remains unrun in this build pass.
