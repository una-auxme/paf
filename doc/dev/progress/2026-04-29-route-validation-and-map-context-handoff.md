# Route Validation And Map Context Handoff

Created: 2026-04-29T08:50:23+02:00
Last updated: 2026-04-29T08:50:23+02:00

## Motivation

Capture the current chat state as a handoff and progress snapshot for the active branch work. The session currently spans three threads:

- deterministic ROS2 route validation and startup/runtime synchronization,
- lightweight route metrics and evidence collection for intersection stops and frame-barrier fallbacks,
- richer intermediate-layer map queries so planning and perception can ask for adjacent-lane presence and traversability.

## Sources inspected

- `agents.md`
- `.github/instructions/docs-and-reasoning.instructions.md`
- current chat state and compacted conversation summary from 2026-04-28 to 2026-04-29
- `build/docker-compose.carla.cuda.yaml`
- `code/paf_common/paf_common/route_metrics.py`
- `code/control/control/vehicle_controller.py`
- `code/planning/planning/behavior_agent/behaviors/intersection.py`
- `code/agent/agent/data_management_node.py`
- `code/agent/agent/startup_coordinator.py`
- `code/leaderboard_launcher/leaderboard_launcher/paf_agent_base.py`
- `code/leaderboard_launcher/scripts/launch_leaderboard.sh`
- `code/planning/planning/global_planner/global_planner_node.py`
- `code/test/run_test.py`
- `code/test/test_route_metrics.py`
- `code/test/test_deterministic_sync.py`
- `code/mapping/mapping_common/map.py`
- `code/mapping/test/test_mapping_common/test_map.py`
- `code/mapping/README.md`
- planning call sites in `lane_change.py`, `overtake.py`, and `leave_parking_space.py`
- runtime evidence from `/internal_workspace/log/ros/agent.log`, `/internal_workspace/simulation_results.json`, and `/tmp/paf_route_metrics.json`

## Progress Summary

### 1. Route metrics instrumentation is in place

The branch now records route-level evidence for two concrete behaviors:

- `planning.intersection.unnecessary_stop_candidates`
- `control.vehicle_controller.frame_barrier_fallback_frames`

The shared helper lives in `code/paf_common/paf_common/route_metrics.py`. Metrics are reset before route runs and merged back into leaderboard checkpoint output afterward.

### 2. ROS2 leaderboard startup deadlocks were removed

The ROS2 route path was pushed from repeated agent setup failure into actual route execution. The main fixes already applied in this chat were:

- preserve the CARLA CUDA Vulkan ICD override in `build/docker-compose.carla.cuda.yaml`,
- publish startup readiness earlier from `data_management_node.py`,
- reduce startup coordinator requirements to a route-independent minimum in `startup_coordinator.py`,
- locally override the ROS2 leaderboard wrapper in `paf_agent_base.py` so the bridge launches with the hero ego role and avoids constructor-time waits that deadlocked before the world was ticking,
- reset, merge, and print route metrics in `launch_leaderboard.sh`.

### 3. Live route validation now reaches route execution but not route completion

The authoritative ROS2 validation path is `leaderboard.dev`, not the legacy ROS1 `leaderboard.test` harness.

The last live validation run reached `> Running the route`, registered sensors in the checkpoint, and showed active CARLA ticks and agent wallclock/game-time output. That is enough to confirm that the original startup deadlocks were removed.

The remaining runtime blocker is later in the stack:

- `PrePlanner` in `code/planning/planning/global_planner/global_planner_node.py` repeatedly logs `Waiting for agent position to stabilize`,
- no global trajectory is published,
- the agent keeps returning timestamp `0` vehicle commands,
- the route therefore does not yet produce a meaningful completed-drive metrics snapshot.

### 4. Intermediate-layer map knowledge was extended

`mapping_common.map.MapTree` now exposes a richer lane-query surface:

- `LanePresence`
- `LaneContext`
- `AdjacentLaneContext`
- `MapTree.get_lane_presence()`
- `MapTree.get_lane_context()`
- `MapTree.get_adjacent_lane_context()`

This separates three cases that were previously collapsed together by `is_lane_free()` call sites:

- adjacent lane exists and is free,
- adjacent lane exists but is blocked,
- adjacent lane is absent or lane availability is unknown.

At this point, the new API exists in the intermediate layer and is documented/tested, but downstream planning behaviors still need to consume it.

### 5. Branch-review meta-status

The higher-level branch review work is partially progressed:

- branch delta inspected,
- core guidance docs read,
- branch narrative docs partially read,
- branch goal summary still pending.

## Validation Completed

- Focused deterministic-sync regression tests passed earlier in the session: `code/test/test_deterministic_sync.py` (`8 passed`)
- Narrow ROS2 package builds passed earlier in the session for the touched route-validation packages
- `colcon build --symlink-install --packages-select mapping` passed in `build-agent-dev-1`
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest /workspace/code/mapping/test/test_mapping_common/test_map.py` passed (`3 passed`)
- Narrow Ruff checks for the touched mapping files passed via `python3 -m ruff check ...` and `python3 -m ruff format --check ...`

## Open Blockers And Risks

- `leaderboard.dev` still does not complete a real driving route end to end.
- `global_planner_node.py` stabilization logic is the current local bottleneck.
- The last live bridge logs still indicated `synchronous_mode_wait_for_vehicle_control_command: False`, which is likely invalid for the intended deterministic frame-barrier contract.
- Route metrics are implemented but not yet proven by a completed ROS2 route that actually drives through the environment.
- The new lane-context API is not yet wired into lane-change, overtake, or other planning decisions.

## Recommended Next Actions

1. Update `code/leaderboard_launcher/leaderboard_launcher/paf_agent_base.py` so the bridge runs with `synchronous_mode_wait_for_vehicle_control_command=True`, then rerun `leaderboard.dev`.
2. Inspect and likely adjust the startup stabilization rule in `code/planning/planning/global_planner/global_planner_node.py` against the observed live localization stream.
3. Once the route drives, inspect `/tmp/paf_route_metrics.json` and the merged metrics in `/internal_workspace/simulation_results.json`.
4. Consume `MapTree.get_lane_context()` or `get_adjacent_lane_context()` from lane-change and overtake behaviors so absent-lane and blocked-lane cases are handled distinctly.
5. Finish reading the branch narrative documents and write the branch goal summary.

## Handoff Notes

- The latest stuck live route terminals were intentionally killed after collecting enough evidence to prove that route startup had been unblocked.
- The current branch state already contains the route-metrics helper, ROS2 startup fixes, and the new map lane-context query surface.
- If this work resumes later, start from the ROS2 `leaderboard.dev` path and the `global_planner_node.py` stabilization gate rather than revisiting the earlier startup-deadlock surfaces first.
