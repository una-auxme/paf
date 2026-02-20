# Linting

**Summary:** Python linting and formatting are handled by Ruff; Markdown linting uses markdownlint.

- [🐍 Python conventions](#-python-conventions)
  - [Ruff versioning](#ruff-versioning)
  - [Run Ruff inside the agent container](#run-ruff-inside-the-agent-container)
  - [Run Ruff via Docker Compose](#run-ruff-via-docker-compose)
- [💬 Markdown Linter](#-markdown-linter)
- [🚨 Common Problems](#-common-problems)

## 🐍 Python conventions

We use [Ruff](https://docs.astral.sh/ruff/) for both linting and formatting Python code.

Repository configs:

- `ruff.toml`: baseline rules used in CI.
- `ruff-strict.toml`: stricter optional profile for gradual hardening (includes docstring checks and additional maintainability rules).

### Ruff versioning

- The pinned version lives in `build/pins/ruff.env` (kept in sync with `build/.env` via `scripts/update-dotenv.sh`, which VS Code runs on folder open).
- Docker Compose linting (`build/docker-compose.linter.yaml`), the GitHub Action (`.github/workflows/ruff.yml`), and the agent container installation (`build/docker/agent-ros2/scripts/install-python-requirements.sh`) all read that pin so the same version is used everywhere.

### Run Ruff inside the agent container

Helper commands are available in `build/docker/agent-ros2/scripts/devfunctions.bash` and are loaded into interactive shells:

- `ruff.lint`: run `ruff check` with the repository rules.
- `ruff.fix-lint`: run `ruff check --fix`.
- `ruff.check-format`: run `ruff format --check`.
- `ruff.format`: apply formatting with `ruff format`.

Use the strict profile when hardening a package:

```bash
ruff check /workspace/code/<package> --config /workspace/ruff-strict.toml
```

### Run Ruff via Docker Compose

Use the pinned version directly from the host:

```bash
docker compose -f build/docker-compose.linter.yaml up
```

The Compose file mounts the repo into the container and runs Ruff against it.

## ✅ Test lint-adjacent workflow

Use pytest markers to scope feedback loops:

```bash
pytest -m unit
pytest -m integration
pytest -m sim
```

## 💬 Markdown Linter

To enforce unified standards in all markdown files, we use [markdownlint-cli](https://github.com/igorshubovych/markdownlint-cli). More details on it can be found in the according documentation.

## 🚨 Common Problems

- If Ruff fails to start because of a missing version, refresh the environment with `scripts/update-dotenv.sh` to regenerate `build/.env` from the pins file.
