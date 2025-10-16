# CARLA Docker builds

This folder contains the CARLA build helper and a multi-stage Dockerfile used to produce several different images with distinct roles.

Short summary of targets produced by `build_carla.sh`:

- `carla-leaderboard-gpu:2.1` (target `carla`, default `BASE_FLAVOUR=gpu`)
  - Full CARLA runtime for non-NVIDIA GPU hosts (Vulkan/OpenGL stacks).
- `carla-leaderboard-cuda:2.1` (target `carla`, built with `BASE_FLAVOUR=cuda`)
  - Full CARLA runtime tuned for NVIDIA/CUDA hosts; enforces driver requirements.
- `carla-leaderboard-api:2.1` (target `carla-api`)
  - Lightweight image containing only the CARLA Python API (no heavy simulator binary).


Why there are multiple images
- Separation of concerns: simulator binary is large — separate images let you run the simulator on one host and the API/bridge on others without shipping the binary everywhere.
- Faster iteration: rebuild only the target that changed

## How to build locally

From `build/docker/carla`:

### Standard build (sequential)

```bash
./build_carla.sh
```

This script will download the CARLA runtime (if missing) and build the three images sequentially. Expect the operation to take a long time and download several GB.

### Buildx build (faster, parallel, with push/cache support)

For faster builds with better layer reuse and the ability to push to ghcr, use the buildx version:

```bash
./build_carla_buildx.sh
```

**Advanced usage for CI/CD:**

```bash
# Push to a container registry (e.g., GHCR)
REGISTRY=ghcr.io/una-auxme/paf PUSH=1 ./build_carla_buildx.sh

# With cache export for faster subsequent builds
REGISTRY=ghcr.io/una-auxme/paf \
  PUSH=1 \
  CACHE_TO=type=registry,ref=ghcr.io/una-auxme/carla-cache \
  ./build_carla_buildx.sh

# With cache import and export
REGISTRY=ghcr.io/una-auxme/paf \
  PUSH=1 \
  CACHE_FROM=type=registry,ref=ghcr.io/una-auxme/carla-cache \
  CACHE_TO=type=registry,ref=ghcr.io/una-auxme/carla-cache \
  ./build_carla_buildx.sh
```

**Benefits of the buildx version:**
- Single build invocation builds all three targets with maximum layer reuse
- Supports pushing directly to container registries
- Supports cache import/export for dramatically faster CI builds
- Can build for multiple platforms (though CARLA is currently amd64-only)
- Uses Docker Bake for declarative multi-target builds

Tips / optimizations
- Keep the CARLA tarball `carla.tar.gz` next to the Dockerfile to reuse it across builds.
- In CI, use cache export to a registry to speed up subsequent builds significantly.

## Publishing prebuilt images

If you prefer not to build CARLA locally, publish the built images to a container registry (Docker Hub, GHCR). Developers can then pull the prebuilt images.

Example: pull a prebuilt CUDA image from GHCR

```bash
docker pull ghcr.io/<org-or-user>/carla-leaderboard-cuda:2.1
```

CI note: Use `docker/build-push-action` (buildx) to build and push multiple targets from the multi-stage Dockerfile and enable cache export to speed up the build.
