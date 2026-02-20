# AGENTS.md

Guidance for AI coding agents working in this repository.

## 1) Mission and scope

- Keep changes **small, focused, and reversible**.
- Prefer fixing root causes over surface patches.
- Respect existing architecture and naming conventions.
- Do not modify unrelated files.

## 2) Repository map (quick orientation)

- `code/`: main ROS2 Python workspace packages (for example `acting/`, `control/`, `localization/`, `mapping/`, `perception/`, `planning/`).
- `code-ros1/`: legacy ROS1 code kept for reference/migration context.
- `build/`: Docker Compose stacks, pins, and container definitions.
- `doc/`: architecture, development process, and subsystem docs.
- `scripts/`: host helper scripts (for example NVIDIA check and env generation).

## 3) Source of truth for conventions

Before making substantial edits, check:

1. `README.md`
2. `doc/development/README.md`
3. `doc/development/linting.md`
4. `doc/development/git_workflow.md`
5. `ruff.toml`

If guidance conflicts, prefer repository config files and actively used CI/lint settings.

## 4) Environment assumptions

- Primary development is Linux + Docker, typically with NVIDIA GPU support.
- Python target is **3.12** (`ruff.toml`).
- Use containerized workflows when project docs expect them.

## 5) Code change rules

- Preserve public interfaces unless the task explicitly requires API changes.
- Match existing style and file structure in each package.
- Avoid adding new dependencies unless required; if added, update the relevant requirements file in `code/`.
- Do not add license headers or broad refactors unless requested.
- Keep comments/docstrings meaningful; avoid noisy inline commentary.

## 6) Linting and formatting

Python linting/formatting is done with **Ruff**.

- Repo config: `ruff.toml`
- Pinned version: `build/pins/ruff.env`

Run one of:

- VS Code tasks (preferred in this workspace):
	- `Lint python code with ruff`
	- `Lint python code with ruff and apply safe fixes`
	- `Check python code formatting with ruff`
	- `Format python code with ruff`
- Or Compose-based linting from docs:

```bash
docker compose -f build/docker-compose.linter.yaml up
```

## 7) Validation strategy

After changes:

1. Run targeted lint/format checks for touched Python files.
2. Run nearest relevant tests (if present) before broad test runs.
3. If Docker/ROS behavior changed, validate with the closest existing launch/build flow.

Do not attempt to fix unrelated failing tests/lints outside the requested scope.

## 8) ROS and package specifics

- `code/` contains ROS2-era packages; `code-ros1/` is legacy reference.
- Respect package boundaries (`package.xml`, `setup.py`, `setup.cfg` per package).
- Keep message/service interface changes isolated and coordinated with dependents.

## 9) Documentation expectations

- Update docs when behavior, setup, commands, or interfaces change.
- For developer-facing changes, prefer updating docs under `doc/development/` or package README files.
- Keep Markdown concise, structured, and actionable.
- Capture non-trivial architectural decisions in `doc/adr/` using the ADR template.

## 10) Git and PR hygiene

- Use feature-branch style consistent with `doc/development/git_workflow.md`.
- Keep PRs focused; include a brief validation summary.
- Avoid forceful history rewrites unless explicitly requested.

## 11) Safety and secrets

- Never commit credentials, tokens, or machine-specific secrets.
- Treat `.env`-style files as environment-specific unless clearly intended for version control.
- Avoid destructive commands (deleting volumes/data, hard resets) unless explicitly requested.

## 12) Agent behavior checklist

Before finishing, ensure:

- [ ] Request scope is fully addressed.
- [ ] Changes are minimal and localized.
- [ ] Lint/format checks were run (or clearly state why not).
- [ ] Relevant docs were updated when needed.
- [ ] Any remaining risks or manual follow-ups are clearly noted.

## 13) Quality gates and contributor contract

- Follow `doc/development/developer_contract.md` for PR assumptions, validation summary, and known gaps.
- Use `pytest` markers defined in `pytest.ini` (`unit`, `integration`, `sim`) to scope validation.
- Use `ruff-strict.toml` for incremental hardening of packages (docstring and maintainability checks).

