# WP1: Documentation and Interface Consolidation

Created: 2026-05-01
Last updated: 2026-05-01

## Motivation

Address the highest-priority improvement areas from `doc/dev_talks/paf25/future_work.md` — specifically WP1 documentation and interface consolidation. Focus on fixing stale references, TODO placeholder descriptions, and ROS1-era terminology before any feature expansion.

## Changes

### Fixed stale package references

- `doc/mapping/README.md`: Replaced stale `mapping_visualization` package references with current `mapping/mapping/visualization.py` location (visualization was merged into the mapping package)
- `doc/mapping/README.md`: Fixed stale `./src/` → `./mapping/` and `./tests/` → `./test/` path references
- `doc/general/architecture_current.md`: Fixed stale `init_mapping` topic references → `init_data` (the actual topic)

### Fixed ROS1-era terminology

- `doc/mapping/README.md`: `catkin_make` → `colcon build` (3 occurrences)
- `code/mapping/mapping_common/__init__.py`: `catkin_make` → `colcon build`
- `doc/mapping/generated/mapping_common/index.md`: `catkin_make` → `colcon build`
- `doc/mapping/generated/mapping_common/README.md`: (symlink to index.md, covered)

### Fixed TODO package descriptions

Replaced `"TODO: Package description"` in 7 `setup.py` files with meaningful descriptions:

- `code/agent/setup.py`
- `code/control/setup.py`
- `code/acting/setup.py`
- `code/localization/setup.py`
- `code/leaderboard_launcher/setup.py`
- `code/planning/setup.py`
- `code/perception/setup.py`

## Validation

- All changed `.py` files pass `get_errors` check (no errors found)
- No lint errors introduced (only string changes in setup.py descriptions, docstring changes in __init__.py)

## Not included (deferred)

- `doc/perception/experiments/` Dockerfile with `catkin_make` — archival content
- `doc/acting/discontinued/` and `doc/planning/discontinued/` CMakeLists references — intentionally preserved historical artifacts
- Remaining `doc/research/overhaul25/` references — research docs, not active architecture
- The generated docs will fully sync on next `pydoc-markdown` run

## Follow-up

- Run `pydoc-markdown` to regenerate generated docs from the fixed source
- Next: WP2 (Radar Motion Quality) or WP5 (Automated Testing)
