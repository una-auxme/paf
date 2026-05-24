# Dev Container Workflow

- [1) Goal](#1-goal)
- [2) Quick Start](#2-quick-start)
- [3) What The Repo Config Does](#3-what-the-repo-config-does)
- [4) Host Auth Workflow](#4-host-auth-workflow)
- [5) Common Auth Tasks](#5-common-auth-tasks)
- [6) Running PAF Inside The Container](#6-running-paf-inside-the-container)
- [7) Troubleshooting](#7-troubleshooting)

## 1) Goal

This repository already ships a compose-backed dev container in [.devcontainer/devcontainer.json](../../.devcontainer/devcontainer.json).
The recommended day-to-day workflow is:

1. Keep the repository on the host.
2. Use VS Code on the host for GitHub, Copilot, and browser-based sign-in.
3. Reopen the workspace in the `agent-dev` container for ROS2, Ruff, and runtime tooling.

This avoids the most common OAuth pain points of the older "everything happens inside the container" model while keeping the runtime environment aligned with the repository's Docker setup.

## 2) Quick Start

1. Open the repository root in VS Code on the host.
2. Install the recommended extensions from [.vscode/extensions.json](../../.vscode/extensions.json).
3. Sign in on the host for the services you need first:
   - GitHub in VS Code
   - GitHub Copilot in VS Code
4. Run `Dev Containers: Reopen in Container`.
5. Select `PAF Agent Dev (CUDA)` if VS Code prompts for a configuration.
6. Inside the container, verify the toolchain:

```bash
ros2 --version
ruff --version
```

If you only need the container stack without reopening VS Code, you can still use:

```bash
bash scripts/dev-up.sh
```

## 3) What The Repo Config Does

The current dev container configuration uses the existing development compose stack instead of inventing a separate image.

- The configuration file is [.devcontainer/devcontainer.json](../../.devcontainer/devcontainer.json).
- It attaches to the `agent-dev` service from [build/docker-compose.dev.cuda.yml](../../build/docker-compose.dev.cuda.yml).
- The workspace inside the container is `/workspace`.
- `initializeCommand` runs `scripts/update-dotenv.sh` on the host before the container starts.

That `.env` refresh is important because it updates host user IDs and, in headless or SSH-forwarded sessions, writes `RENDER_OFFSCREEN=-RenderOffScreen` so CARLA can start without a local desktop renderer.

The dev container only starts `agent-dev` on attach. Runtime services such as CARLA stay opt-in and can be launched when needed via scripts or VS Code tasks.

## 4) Host Auth Workflow

The supported auth model is split intentionally:

- VS Code UI sign-in should happen on the host.
- Browser-based OAuth should happen on the host.
- The repository files are bind-mounted into the container, but credentials should not be baked into images.

This is the path that works best for:

- GitHub Pull Requests extension
- GitHub Copilot
- GitHub Copilot Chat

When these extensions are installed and authenticated on the host, VS Code can continue to use them while you edit the mounted workspace inside the container.

## 5) Common Auth Tasks

### GitHub and Copilot in VS Code

1. Open the repository on the host in VS Code.
2. Sign in to GitHub through the Accounts menu.
3. Sign in to Copilot through the Accounts menu or extension prompt.
4. Reopen in container only after host sign-in is complete.

### GitHub CLI

For CLI usage, prefer a host terminal when possible:

```bash
gh auth login --web
```

If you specifically need `gh` inside the container, use an explicit device or web flow there as well. That is still a container-local CLI credential, so prefer host-side auth unless container-local CLI access is required.

### GHCR

For GHCR pushes or pulls from the host:

```bash
gh auth token | docker login ghcr.io -u <github-user> --password-stdin
```

Alternatively use a fine-scoped PAT if your workflow requires it.

## 6) Running PAF Inside The Container

Once attached to the container, the existing repository workflows continue to apply.

Useful commands:

```bash
source /internal_workspace/dev.bashrc
dep.check
ruff check /workspace/code/
ruff format /workspace/code/ --check
```

Useful VS Code tasks:

- `Update build/.env file`
- `Start dev container stack (CUDA)`
- `Run host smoke tests`
- `Run ROS-backed unit tests (dev container)`
- `Pre-PR quality check`

## 7) Troubleshooting

### Dev container starts but CARLA later fails in headless sessions

Re-run:

```bash
bash scripts/update-dotenv.sh
```

Then verify that `build/.env` contains `RENDER_OFFSCREEN=-RenderOffScreen` for SSH-forwarded or otherwise headless sessions.

### GitHub or Copilot prompts still appear broken inside the container

Make sure the sign-in was completed on the host before reopening in container. The supported path is host-auth plus containerized tooling, not a browser installed inside the container.

### `gh` works on the host but not in the container

That is expected unless you explicitly log in inside the container as well. Prefer host-side `gh` usage for authentication-heavy flows.

### GPU or ROS tooling is missing after attach

Check the usual prerequisites:

```bash
bash scripts/check-nvidia.sh
bash scripts/update-dotenv.sh
```

Then rebuild or reopen the container if needed.
