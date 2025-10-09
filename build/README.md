# Build / Docker compose overview

This document explains the standard build/run entrypoint for local development and describes the other docker-compose files under `build/`. The normal development entrypoint is `build/docker-compose.dev.cuda.yml` (use this when you have an NVIDIA GPU and want CUDA-enabled containers). A mermaid process flow for the CUDA dev entrypoint is included below.

## Quick start (recommended: CUDA desktop)

From the repository root:

```bash
# Export user mappings (recommended) and bring up the dev stack (CUDA flavor)
PAF_USERNAME=$(id -u -n) PAF_UID=$(id -u) PAF_GID=$(id -g) \
  docker compose -f build/docker-compose.dev.cuda.yml up --build
```

Notes:
- `PAF_USERNAME`, `PAF_UID`, `PAF_GID` are used by the build to create a matching user inside containers.
- If you have an NVIDIA GPU, make sure your host driver meets the project requirement (see below). If your driver does not meet the minimum, the nvidia container runtime may refuse to start GPU containers.
- For convenience the repo includes a small checker: `scripts/check-nvidia.sh` and a VS Code automatic task that runs it on folder open.

## NVIDIA driver / CUDA notes

- This project uses images targeting CUDA 12.x. Those images require an NVIDIA driver with a matching ABI. By default the build sets a constraint in the agent images:

  - `NVIDIA_REQUIRE_DRIVER="cuda>=12.0 driver>=550"`

- In short: NVIDIA driver >= 550 is recommended for the CUDA images used here. If you can't upgrade the host driver you have three options:
  1. Use the non-CUDA compose files (see table below) that target CPU or non-CUDA GPU stacks.
 2. Rebuild images based on an older CUDA base (advanced).
 3. Temporarily disable the runtime check (not recommended for production) by setting `NVIDIA_DISABLE_REQUIRE=1` in the compose override for the affected service — this may still fail at runtime if ABI/features are missing.

## Dockerfile notes (agent image)

See `build/docker/agent-ros2/Dockerfile`. Important build-time args and environment variables:

- `ARG BASE_FLAVOUR` and `FROM agent-base-${BASE_FLAVOUR}` – controls the base stack (generic/gpu/rocm/cuda).
- `ARG NVIDIA_DRIVER_VERSION` – used when building the CUDA base image to pin driver-related packages.
- The CUDA base sets `NVIDIA_REQUIRE_DRIVER` so the runtime enforces driver constraints.

If you change `BASE_FLAVOUR` you must rebuild the appropriate images (or use a different compose file that matches that flavour).

## Which compose file should I use? (overview)

The project contains several docker-compose files under `build/`. The table below summarizes the most commonly used ones.

| Compose file | Purpose | Notes |
|---|---|---|
| `docker-compose.dev.cuda.yml` | Primary developer stack for machines with NVIDIA GPUs and CUDA support | Builds CUDA-enabled images (default entrypoint). Requires host NVIDIA driver >= 550 for CUDA 12.x images.
| `docker-compose.dev.base.yml` | Shared base compose for dev services | Used by the per-flavour dev compose files as a building block.
| `docker-compose.dev.intel.yml` | Dev stack targeting Intel integrated graphics (no CUDA) | Uses CPU/OpenGL/Vulkan drivers; suitable for laptops with Intel iGPU.
| `docker-compose.dev.rocm.yml` | Dev stack targeting AMD ROCm (compute on AMD GPUs) | For AMD compute; only use if you have a ROCm-capable setup.
| `docker-compose.dev.wsl.yml` | WSL-specific dev compose | Tweaks for Windows Subsystem for Linux environments.
| `docker-compose.carla.cuda.yaml` | CARLA simulator configuration for CUDA hosts | Builds/starts CARLA simulator images tuned for NVIDIA.
| `docker-compose.carla.gpu.yaml` | CARLA simulator configuration for non-CUDA GPU hosts | GPU but not CUDA (e.g., Vulkan-only workflows).
| `docker-compose.carla.base.yaml` | CARLA minimal / headless base config | Smaller footprint; useful for CI or headless runs.
| `docker-compose.carla.wsl.yaml` | CARLA tweaks for WSL | Use when running under WSL2 on Windows.
| `docker-compose.leaderboard.yaml` | Leaderboard / evaluation stack for CARLA | Runs scoring/benchmark services.
| `docker-compose.leaderboard-distributed.yaml` | Distributed leaderboard configuration | Multi-node / distributed evaluation.
| `docker-compose.test.yaml` | CI/test compose used by local tests | Useful for reproducing CI locally.
| `docker-compose.linter.yaml` | Linting and static checks | Runs linters and formatters in containers.
| `docker-compose.cicd.yaml` | Continuous integration helpers | CI-specific services and test runners.
| `docker-compose.docs.yaml` | Documentation build environment | Build docs in containerized environment.

If you're unsure, start with `docker-compose.dev.cuda.yml` on an NVIDIA desktop, or `docker-compose.dev.base.yml` + the flavour file for non-NVIDIA setups.

## Build process (step-by-step)

1. Prepare host
   - Ensure Docker and docker-compose (v2) are installed.
   - If using NVIDIA GPU: install NVIDIA drivers and the NVIDIA Container Toolkit.
   - (Optional) Run `scripts/check-nvidia.sh` to confirm driver >= 550.

2. Build CARLA (if using CARLA simulator)
   - The CARLA build helper is `build/docker/carla/build_carla.sh` and produces local images named like `carla-leaderboard` & `carla-leaderboard-ros-bridge`.
   - This step downloads the CARLA runtime (~7–8 GB) and can take a while.

3. Build agent images
   - The agent images are built by the compose files when you run `docker compose up --build`.
   - Alternatively, build a single image manually (example):

```bash
# Build agent-dev (CUDA flavour)
docker build \
  --build-arg BASE_FLAVOUR=cuda \
  --build-arg PAF_USERNAME=$(id -u -n) \
  --build-arg PAF_UID=$(id -u) \
  --build-arg PAF_GID=$(id -g) \
  -t paf/agent-dev:local -f build/docker/agent-ros2/Dockerfile build/docker
```

4. Start the dev stack
   - Example (detached):

```bash
PAF_USERNAME=$(id -u -n) PAF_UID=$(id -u) PAF_GID=$(id -g) \
  docker compose -f build/docker-compose.dev.cuda.yml up --build -d
```

5. Attach IDE / developer workflow
   - Attach your editor (VS Code Remote - Containers or simply attach to running `agent-dev` container).
   - The agent-dev container exposes a persistent dev environment and links `/workspace` to your repo.

## Troubleshooting

- "pull access denied for carla-leaderboard-ros-bridge": run `build/docker/carla/build_carla.sh` to build the CARLA images locally. The build downloads CARLA runtime artifacts (~7–8 GB).
- "nvidia-container-cli: requirement error: driver>=550": update your NVIDIA driver or switch to a non-CUDA dev compose (see table). Use `scripts/check-nvidia.sh` to diagnose.
- If a particular service refuses to start with GPU errors, try setting the env override `NVIDIA_DISABLE_REQUIRE=1` for that service (quick workaround; not guaranteed to work).

## Process flow (CUDA dev entrypoint)

```mermaid
flowchart TD
  A[Start: run docker compose -f build/docker-compose.dev.cuda.yml up --build]
  B[Check host prerequisites: Docker, nvidia driver >= 550?]
  C{NVIDIA present?}
  D[Run build scripts: build CARLA images (build/docker/carla/build_carla.sh)]
  E[Build agent images (docker/agent-ros2/Dockerfile) with BASE_FLAVOUR=cuda]
  F[Compose creates networks & volumes]
  G[Start services: carla-simulator, carla-ros-bridge, agent-dev, ...]
  H[Attach dev tools: VS Code, shells, tests]
  I[Run experiments / development loop]

  A --> B --> C
  C -- yes --> D --> E --> F --> G --> H --> I
  C -- no --> E --> F --> G --> H --> I
  %% If driver check fails there is an optional manual override
  G -.-> |driver too old| J[Set NVIDIA_DISABLE_REQUIRE=1 (workaround)] -.-> G
```

## Contact / notes

If you change base images or CUDA versions, update the `NVIDIA_REQUIRE_DRIVER` setting in `build/docker/agent-ros2/Dockerfile` and document the change here. When altering CARLA versions, update the CARLA build helper and checks in `build/docker/carla/`.

Happy hacking — open an issue or PR if you want the build to support additional host configurations or to automate more checks.
