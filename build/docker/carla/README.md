# CARLA Docker builds

This folder contains the CARLA build helper and a multi-stage Dockerfile used to produce several different images with distinct roles.

Short summary of targets produced by `build_carla.sh`:

- `carla-leaderboard-gpu:2.1` (target `carla`, default `BASE_FLAVOUR=gpu`)
  - Full CARLA runtime for non-NVIDIA GPU hosts (Vulkan/OpenGL stacks).
- `carla-leaderboard-cuda:2.1` (target `carla`, built with `BASE_FLAVOUR=cuda`)
  - Full CARLA runtime tuned for NVIDIA/CUDA hosts; enforces driver requirements.
- `carla-leaderboard-api:2.1` (target `carla-api`)
  - Lightweight image containing only the CARLA Python API (no heavy simulator binary).
- `carla-leaderboard-ros-bridge:2.1` (target `carla-ros-bridge`)
  - Image with ROS/leaderboard/scenario-runner and the ROS bridge; used for scoring, bridges and runner processes.

Why there are multiple images
- Separation of concerns: simulator binary is large — separate images let you run the simulator on one host and the API/bridge on others without shipping the binary everywhere.
- Faster iteration: rebuild only the target that changed (the ros-bridge rebuilds faster than re-packing the simulator tarball).

How to build locally

From `build/docker/carla`:

```bash
./build_carla.sh
```

This script will download the CARLA runtime (if missing) and build the four images. Expect the operation to take a long time and download several GB.

Tips / optimizations
- Use Docker Buildx to export multiple images in a single build invocation to reduce duplicated downloads and layer churn in CI.
- Keep the CARLA tarball `CARLA_Leaderboard_2.0.tar.xz` next to the Dockerfile to reuse it across builds.

If you want, I can add a `buildx` version of `build_carla.sh` that performs a single multi-target build and pushes/cache-export images for faster CI runs.
