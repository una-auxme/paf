Created: 2026-04-30T22:29:21+02:00
Last updated: 2026-04-30T22:54:59+02:00

# Route Sync Bridge Progress

## Motivation

Continue the route-validation implementation from
`2026-04-29-route-validation-and-map-context-handoff.md`, starting with the
recorded blocker that the overridden ROS2 leaderboard bridge did not pass
`synchronous_mode_wait_for_vehicle_control_command=True`.

## Sources inspected

- `doc/dev/progress/2026-04-29-route-validation-and-map-context-handoff.md`
- `code/leaderboard_launcher/leaderboard_launcher/paf_agent_base.py`
- `code/leaderboard_launcher/launch/ros_bridge.dev.xml`
- `code/test/test_deterministic_sync.py`
- `code/planning/planning/global_planner/global_planner_node.py`

## Current chat state

The wrapper launch parameters in `paf_agent_base.py` now explicitly set
`synchronous_mode_wait_for_vehicle_control_command` to `True`, matching the
existing `ros_bridge.dev.xml` contract and the control frame-barrier design.

The deterministic sync regression test now checks both launch surfaces:

- the XML `synchronous_mode_wait_for_vehicle_control_command` default,
- the Python leaderboard wrapper parameter passed to `ROSLauncher.run()`.

The `PrePlanner` startup position gate now uses a pure
`PositionStabilityGate`. It keeps the existing stable-window behavior, but adds
a bounded unstable sample window through
`position_stabilization_max_unstable_samples` so route preplanning cannot wait
forever on a drifting localization stream. The existing
`distance_spawn_to_first_wp` check still rejects poses that are too far away
from the route start before a global trajectory is generated.

`agents.md` now also records the operational cleanup rule that any CARLA run
must be stopped, including the simulator, compose stack, and related
route-validation processes, before handoff or completion.

The follow-up issue sweep handled the checkout-safe subset of open GitHub
issues:

- local Ruff/code-format issues from `code/` were fixed,
- stale GitHub Action versions in `drive.yml` and `markdownlint.yml` were
  updated,
- Docker Compose wording was updated where it referred to the old binary name
  rather than compose file names,
- global-planner update notifications now use transient-local QoS so late
  subscribers can observe route-data availability.

## Validation

- Red check: the focused deterministic sync contract test failed before the
  Python wrapper parameter was added.
- Green check:
  `uv run --with pytest==9.0.2 python -m pytest code/test/test_deterministic_sync.py -q`
  passed with `8 passed`.
- Red check: the focused planning regression test failed while
  `planning.global_planner.position_stability` did not exist.
- Green check:
  `uv run --with pytest==9.0.2 --with numpy python -m pytest code/test/test_planning_regression.py -q`
  passed with `3 passed`.
- Green check:
  `uv run --with ruff==0.14.8 ruff check code` passed.
- Green check:
  `uv run --with ruff==0.14.8 ruff format --check code` passed.
- Green check:
  `uv run --with pytest==9.0.2 --with numpy python -m pytest code/test -m unit -q`
  passed with `27 passed`.

## Remaining blockers

- `leaderboard.dev` still needs a live rerun with CARLA to verify the bridge
  now waits for vehicle-control commands in the actual route.
- Route metrics still need proof from a completed ROS2 drive.
- Planning behaviors still need to consume the new `MapTree` lane-context API.
- Several open GitHub issues remain intentionally unclaimed because they need
  CARLA validation, perception/planning research, or larger architecture work.

## Recommended next actions

1. Run the documented `leaderboard.dev` route validation path with CARLA.
2. Inspect bridge logs for
   `synchronous_mode_wait_for_vehicle_control_command: True`.
3. If the route still does not publish a global trajectory, inspect the
   `distance_spawn_to_first_wp` check and the OpenDRIVE/global-plan service
   responses in `PrePlanner.process_global_plan()`.
