# Testing Strategy

## Goals

- Fast feedback for local development.
- Confidence in subsystem integration.
- Clear separation between deterministic and simulation-heavy tests.

## Test levels

- `unit`: Isolated logic tests with no external services and no ROS graph dependency.
- `integration`: Multi-module behavior tests using realistic data paths.
- `sim`: End-to-end or high-fidelity simulation tests (e.g., CARLA).

## Markers and commands

Repository markers are defined in `pytest.ini`.

- Run host smoke tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest code/test -m unit`
- Run unit tests in the current environment: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -m unit`
- Run integration tests: `pytest -m integration`
- Run simulation tests: `pytest -m sim`
- Run all tests with coverage: `pytest --cov=code --cov-report=term-missing`

## Host smoke tests vs ROS-backed unit tests

Use two different fast loops:

- Host smoke tests: `code/test` only. These must stay runnable outside the dev container and should not depend on `/workspace`, sourced ROS overlays, or generated interfaces.
- ROS-backed unit tests: package-local tests such as `code/perception/tests`, `code/mapping/test`, and `code/planning/test`. These run inside the dev container after building the required package closure.

Example ROS-backed unit test loop inside the dev container:

```bash
source /internal_workspace/dev.bashrc
colcon build --symlink-install --continue-on-error --packages-up-to mapping perception planning
devsource
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest /workspace/code/perception/tests /workspace/code/mapping/test /workspace/code/planning/test -m unit
```

## Live GNSS validation during CARLA runs

Use the live GNSS projection check only while `leaderboard.test` is already running in the dev container.

- Task: `Validate live GNSS projection`
- Script: `bash scripts/validate-gnss-projection.sh`

The script starts a sidecar `gps_transform` with `position_use_ground_truth=false`, waits for `/odometry/gps_projection_check`, and compares the output against CARLA ground truth for the live `hero` actor. If `pyproj` is missing in the dev container, it first runs the dependency synchronization workflow.

Optional thresholds can be supplied through environment variables:

- `PAF_GNSS_MEAN_THRESHOLD_M`
- `PAF_GNSS_MAX_THRESHOLD_M`

## Submodule strategy

For each package under `code/<package>`:

1. Keep `unit` tests near core algorithms and utilities.
2. Add `integration` tests where package boundaries are crossed.
3. Reserve `sim` tests for behavior that needs runtime/simulator fidelity.
4. Prefer deterministic fixtures and fixed random seeds.

## CI recommendation (phased)

- On every PR: run host smoke tests on `ubuntu-latest`.
- On every PR: run ROS-backed mapping/perception/planning unit tests in the agent container on the self-hosted build runner.
- On every PR: run strict Ruff for starter packages that have opted in.
- On merge to main or scheduled runs: run `integration` tests.
- Nightly or hardware-backed pipeline: run `sim` tests.
