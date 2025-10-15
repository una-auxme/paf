#!/usr/bin/env bash
set -e

# build_carla_buildx.sh
echo "Buildx version of build_carla.sh that performs a single multi-target build"
echo "with push and cache export capabilities for faster CI runs."

# Usage:
#   Local build (load to docker):
#     ./build_carla_buildx.sh
#
#   CI build with push to registry:
#     REGISTRY=ghcr.io/una-auxme PUSH=1 ./build_carla_buildx.sh
#
#   CI build with cache export:
#     REGISTRY=ghcr.io/una-auxme PUSH=1 CACHE_TO=type=registry,ref=ghcr.io/una-auxme/carla-cache ./build_carla_buildx.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cd "${SCRIPT_DIR}"

# Configuration
REGISTRY="${REGISTRY:-ghcr.io/una-auxme}"  # e.g., ghcr.io/una-auxme or docker.io/yourorg
TAG="${TAG:-2.1}"
PUSH="${PUSH:-1}"
PLATFORMS="${PLATFORMS:-linux/amd64}"
CACHE_FROM="${CACHE_FROM:-}"
CACHE_TO="${CACHE_TO:-}"

# Basic checks
command -v docker >/dev/null 2>&1 || { echo "[error] Docker not found."; exit 1; }

# Ensure buildx is available
if ! docker buildx version &>/dev/null; then
  echo "Error: docker buildx is not available. Please install Docker Buildx."
  exit 1
fi

if [ "$PUSH" = "1" ]; then
  # Resolve GH_TOKEN
  if [ -z "${GH_TOKEN:-}" ]; then
    if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
      GH_TOKEN="$(gh auth token)"
      echo "[info] Using token from gh CLI."
    else
      echo "[error] GH_TOKEN not set and gh CLI not logged in."
      echo "        Export GH_TOKEN (PAT with write:packages) or run: gh auth login"
      exit 1
    fi
  fi

  # Resolve GH_USER (token owner)
  if [ -z "${GH_USER:-}" ]; then
    if command -v jq >/dev/null 2>&1; then
      GH_USER="$(curl -fsSL -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | jq -r .login)"
    else
      GH_USER="$(curl -fsSL -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | sed -n 's/.*"login": *"\([^"]*\)".*/\1/p')"
    fi
    if [ -z "$GH_USER" ] || [ "$GH_USER" = "null" ]; then
      echo "[error] Could not determine token owner username via GitHub API."
      echo "        Set GH_USER explicitly: export GH_USER=<your-github-username>"
      exit 1
    fi
  fi

  # Optional scope hint (classic PATs only)
  if command -v curl >/dev/null 2>&1; then
    SCOPES="$(curl -sI -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | tr -d '\r' | sed -n 's/^x-oauth-scopes: *//Ip')"
    if [ -n "$SCOPES" ] && ! echo "$SCOPES" | grep -q 'write:packages'; then
      echo "[warn] Token appears to be missing write:packages (scopes: $SCOPES)"
      echo "       If your org enforces SSO, ensure the token is SSO-granted."
    fi
  fi

  # Login
  echo "$GH_TOKEN" | docker login ghcr.io -u "$GH_USER" --password-stdin
  echo "[info] Logged in to ghcr.io as $GH_USER"
fi

# Download CARLA tarballs if missing
if [ ! -f ./carla.tar.gz ]; then
  echo "Downloading CARLA runtime..."
  wget --progress=bar https://tiny.carla.org/carla-0-9-16-linux -O ./carla.tar.gz
fi

if [ ! -f ./carla-additional-maps.tar.gz ]; then
  echo "Downloading CARLA additional maps..."
  wget --progress=bar https://tiny.carla.org/additional-maps-0-9-16-linux -O ./carla-additional-maps.tar.gz
fi

# Create or use existing buildx builder
BUILDER_NAME="carla-builder"
if ! docker buildx inspect "$BUILDER_NAME" &>/dev/null; then
  echo "Creating buildx builder: $BUILDER_NAME"
  docker buildx create --name "$BUILDER_NAME" --driver docker-container --use
else
  echo "Using existing buildx builder: $BUILDER_NAME"
  docker buildx use "$BUILDER_NAME"
fi

# Determine output mode
if [ "$PUSH" = "1" ]; then
  if [ -z "$REGISTRY" ]; then
    echo "Error: PUSH=1 requires REGISTRY to be set (e.g., REGISTRY=ghcr.io/una-auxme)"
    exit 1
  fi
  OUTPUT="--push"
  echo "Mode: Push to registry $REGISTRY"
else
  OUTPUT="--load"
  echo "Mode: Load to local docker"
  if [ "$PLATFORMS" != "linux/amd64" ]; then
    echo "Warning: --load only supports single platform. Setting PLATFORMS=linux/amd64"
    PLATFORMS="linux/amd64"
  fi
fi

# Build cache arguments
CACHE_ARGS=""
if [ -n "$CACHE_FROM" ]; then
  CACHE_ARGS="$CACHE_ARGS --cache-from=$CACHE_FROM"
  echo "Using cache from: $CACHE_FROM"
fi
if [ -n "$CACHE_TO" ]; then
  CACHE_ARGS="$CACHE_ARGS --cache-to=$CACHE_TO"
  echo "Exporting cache to: $CACHE_TO"
fi

# Construct image tags
if [ -n "$REGISTRY" ]; then
  GPU_IMAGE="${REGISTRY}/carla-leaderboard-gpu:${TAG}"
  CUDA_IMAGE="${REGISTRY}/carla-leaderboard-cuda:${TAG}"
  API_IMAGE="${REGISTRY}/carla-leaderboard-api:${TAG}"
else
  GPU_IMAGE="carla-leaderboard-gpu:${TAG}"
  CUDA_IMAGE="carla-leaderboard-cuda:${TAG}"
  API_IMAGE="carla-leaderboard-api:${TAG}"
fi

echo ""
echo "Building CARLA images with buildx..."
echo "  GPU image:  $GPU_IMAGE"
echo "  CUDA image: $CUDA_IMAGE"
echo "  API image:  $API_IMAGE"
echo "  Platforms:  $PLATFORMS"
echo ""

# Build all targets in a single multi-target build using bake
# This approach maximizes layer reuse across targets
cat > docker-bake.hcl <<EOF
variable "REGISTRY" {
  default = "${REGISTRY}"
}

variable "TAG" {
  default = "${TAG}"
}

variable "PLATFORMS" {
  default = "${PLATFORMS}"
}

group "default" {
  targets = ["carla-gpu", "carla-cuda", "carla-api"]
}

target "carla-gpu" {
  context    = "../../.."
  dockerfile = "build/docker/carla/Dockerfile"
  target     = "carla"
  platforms  = split(",", PLATFORMS)
  tags       = [REGISTRY != "" ? "\${REGISTRY}/carla-leaderboard-gpu:\${TAG}" : "carla-leaderboard-gpu:\${TAG}"]
  args = {
    BASE_FLAVOUR = "gpu"
  }
}

target "carla-cuda" {
  context    = "../../.."
  dockerfile = "build/docker/carla/Dockerfile"
  target     = "carla"
  platforms  = split(",", PLATFORMS)
  tags       = [REGISTRY != "" ? "\${REGISTRY}/carla-leaderboard-cuda:\${TAG}" : "carla-leaderboard-cuda:\${TAG}"]
  args = {
    BASE_FLAVOUR = "cuda"
  }
}

target "carla-api" {
  context    = "../../.."
  dockerfile = "build/docker/carla/Dockerfile"
  target     = "carla-api"
  platforms  = split(",", PLATFORMS)
  tags       = [REGISTRY != "" ? "\${REGISTRY}/carla-leaderboard-api:\${TAG}" : "carla-leaderboard-api:\${TAG}"]
}
EOF

# Execute the build
docker buildx bake \
  --pull \
  --file docker-bake.hcl \
  $OUTPUT \
  $CACHE_ARGS

echo ""
echo "✅ Build completed successfully!"
echo ""

if [ "$PUSH" = "1" ]; then
  echo "Images pushed to registry:"
  echo "  - $GPU_IMAGE"
  echo "  - $CUDA_IMAGE"
  echo "  - $API_IMAGE"
else
  echo "Images loaded to local docker:"
  echo "  - $GPU_IMAGE"
  echo "  - $CUDA_IMAGE"
  echo "  - $API_IMAGE"
  echo ""
  echo "To push these images, run with PUSH=1 and set REGISTRY:"
  echo "  REGISTRY=ghcr.io/una-auxme PUSH=1 ./build_carla_buildx.sh"
fi

# Clean up bake file
rm -f docker-bake.hcl
