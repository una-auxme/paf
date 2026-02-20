# Contributor Quickstart (10 minutes)

Use this page for the fastest path from clone to productive development.

- [1) Open and prepare the workspace](#1-open-and-prepare-the-workspace)
- [2) Start the development environment](#2-start-the-development-environment)
- [3) Open in Dev Container](#3-open-in-dev-container)
- [4) Verify local toolchain](#4-verify-local-toolchain)
- [5) Enable pre-commit checks](#5-enable-pre-commit-checks)
- [6) Typical daily workflow](#6-typical-daily-workflow)
- [7) Common problems](#7-common-problems)

## 1) Open and prepare the workspace

1. Clone and open the repository root in VS Code.
2. Install the recommended extensions from `.vscode/extensions.json`.

## 2) Start the development environment

Run:

```bash
bash scripts/dev-up.sh
```

This command:

- updates `build/.env` with your host user IDs,
- checks NVIDIA driver compatibility,
- starts `agent-dev` with `build/docker-compose.dev.cuda.yml`.

## 3) Open in Dev Container

1. Open command palette (`Ctrl+Shift+P`).
2. Run `Dev Containers: Reopen in Container`.
3. Select the `PAF Agent Dev (CUDA)` configuration if prompted.

The devcontainer uses the existing `agent-dev` service and opens `/workspace`.

## 4) Verify local toolchain

Inside the container terminal:

```bash
ros2 --version
ruff --version
```

Optionally run parity checks used in CI/doc workflows:

```bash
ruff check /workspace/code/
ruff format /workspace/code/ --check
docker compose -f build/docker-compose.linter.yaml up
```

## 5) Enable pre-commit checks

Inside the container:

```bash
pip install --user pre-commit
pre-commit install
```

Run all hooks once:

```bash
pre-commit run --all-files
```

## 6) Typical daily workflow

1. Pull latest `main`.
2. Create a feature branch (`<issue-number>-<short-description>`).
3. Implement a small focused change.
4. Run scoped tasks from VS Code:
   - `Lint current Python file with ruff`
   - `Format current Python file with ruff`
   - `Lint active package with ruff`
   - `Dependency check in dev container`
5. Commit only related files and open a focused PR.

## 7) Common problems

- If Ruff version mismatch appears, run `bash scripts/update-dotenv.sh` again.
- If container GPU access fails, re-run `bash scripts/check-nvidia.sh` and review `doc/general/installation.md`.
- If rosdep/pip setup failed during image build, inspect logs inside container:
  - `/internal_workspace/rosdep_install.log`
  - `/internal_workspace/pip_install.log`
- If dependency drift appears after changing `requirements*.txt` or `package.xml`, run:
   - `dep.sync`
   - `devbuild`
