# 2026-05-04 Host testability and control logic consolidation

## Motivating task

Improve repository-wide quality in a consolidation-first way by strengthening the fastest validation loop, increasing testability, and reducing branch-heavy ROS-node logic that was hard to test outside a built workspace.

## Current chat state

This change set focuses on a realistic high-leverage slice instead of claiming full-repository completion. It improves host smoke-test reproducibility, extracts pure control helpers from ROS nodes, and adds host-runnable unit coverage for the extracted logic.

## Source files and systems inspected

- `agents.md`
- `.agent/PLANS.md`
- `doc/dev_talks/paf25/future_work.md`
- `doc/dev_talks/paf25/improvements_assessment.md`
- `doc/development/testing_strategy.md`
- `.github/workflows/unit-tests.yml`
- `code/control/control/vehicle_controller.py`
- `code/control/control/velocity_controller.py`
- `code/test/test_planning_regression.py`
- `code/test/test_deterministic_sync.py`

## Work completed

- Added `scripts/bootstrap-host-python.sh` to create or refresh a repo-local host tooling environment from committed manifests and to run commands through it.
- Added `scripts/run-host-smoke-tests.sh` so local contributors and CI use the same host smoke-test entrypoint.
- Extracted pure helper modules:
  - `code/control/control/vehicle_control_logic.py`
  - `code/control/control/velocity_control_logic.py`
- Rewired `vehicle_controller.py` and `velocity_controller.py` to use the extracted helpers while preserving their external ROS/CARLA interfaces.
- Added `code/test/test_control_logic.py` with host-runnable coverage for manual override, emergency stop, unstuck steering sign handling, fixed-speed override, PID planning, and PID output mapping.
- Updated `.github/workflows/unit-tests.yml` to run the repository script instead of duplicating bootstrap steps.
- Updated the relevant developer docs for the new host smoke-test/bootstrap path.
- Ignored `.venv/` and `.venv-host/` in `.gitignore`.

## Validation performed

- `bash scripts/run-host-smoke-tests.sh`
- `bash scripts/bootstrap-host-python.sh python -m ruff check code/control/control/vehicle_control_logic.py code/control/control/velocity_control_logic.py code/control/control/vehicle_controller.py code/control/control/velocity_controller.py code/test/test_control_logic.py`
- `bash scripts/bootstrap-host-python.sh python -m ruff format --check code/control/control/vehicle_control_logic.py code/control/control/velocity_control_logic.py code/control/control/vehicle_controller.py code/control/control/velocity_controller.py code/test/test_control_logic.py`

Observed result:

- host smoke tests passed (`42 passed`)
- targeted Ruff lint passed
- targeted Ruff format check passed

## Remaining blockers and follow-ups

- ROS-backed tests for the runtime control nodes are still a useful next step inside the dev container after building the package closure.
- Open issues around radar quality and planning startup behavior were inspected during triage, but not claimed as solved here without direct runtime proof.
- If this consolidation slice is merged, the next best follow-up is to extract and test additional pure helpers from `acting` or add focused ROS-backed control tests in CI.
