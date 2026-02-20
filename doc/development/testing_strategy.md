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

- Run unit tests: `pytest -m unit`
- Run integration tests: `pytest -m integration`
- Run simulation tests: `pytest -m sim`
- Run all tests with coverage: `pytest --cov=code --cov-report=term-missing`

## Submodule strategy

For each package under `code/<package>`:

1. Keep `unit` tests near core algorithms and utilities.
2. Add `integration` tests where package boundaries are crossed.
3. Reserve `sim` tests for behavior that needs runtime/simulator fidelity.
4. Prefer deterministic fixtures and fixed random seeds.

## CI recommendation (phased)

- On every PR: run `unit` tests.
- On merge to main or scheduled runs: run `integration` tests.
- Nightly or hardware-backed pipeline: run `sim` tests.
