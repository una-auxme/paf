# Developer Contract

This page defines the minimum quality bar for all code contributions.

## 1) Before coding

- Confirm scope and assumptions in the issue/PR description.
- Identify impacted package boundaries (`code/<package>/...`).
- Decide required validation level: `unit`, `integration`, or `sim`.

## 2) During coding

- Keep changes small and package-local.
- Avoid public API changes unless required by scope.
- Use structured logging and avoid `print` in runtime nodes.
- Add/adjust tests for behavior changes.

## 3) Before opening PR

- Run lint and format checks:
  - `ruff check /workspace/code/`
  - `ruff format /workspace/code/ --check`
- Run strict lint incrementally (recommended):
  - `ruff check /workspace/code/<package> --config /workspace/ruff-strict.toml`
- Run relevant tests by marker:
  - `pytest -m unit`
  - `pytest -m integration`
  - `pytest -m sim` (when simulation behavior changed)

## 4) PR quality fields (required)

Every PR must include:

- Assumptions and constraints.
- Validation summary (commands + result).
- Known gaps/follow-ups.
- Explicit note if docs were updated or why not.
