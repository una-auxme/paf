#!/bin/bash
# Abort on error
set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

CARLA_PKG_PATH=${SCRIPT_DIR}/CARLA_Leaderboard_2.0.tar.xz
if [ ! -f "${CARLA_PKG_PATH}" ]; then
  wget -O "${CARLA_PKG_PATH}" https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz
fi

DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile"
CONTAINER_IMAGE_NAME="paf-peter-distrobox-rocm"
DISTROBOX_NAME="paf-rocm-full"

HOST_WORKSPACE_DIR="${SCRIPT_DIR}/../../../"
HOST_USER_PACKAGE_DIR="${SCRIPT_DIR}/user-package"
USER_PACKAGE_DIR=/user-package
WORKSPACE_MOUNT="/workspace"

podman build -t ${CONTAINER_IMAGE_NAME} -f "${DOCKERFILE_PATH}" "${SCRIPT_DIR}"
distrobox create \
  --image ${CONTAINER_IMAGE_NAME} \
  --name ${DISTROBOX_NAME} \
  --home ~/Distrobox/${DISTROBOX_NAME} \
  --init-hooks '/init_hook.sh' \
  --volume "${HOST_WORKSPACE_DIR}":${WORKSPACE_MOUNT} \
  -a '--env ZDOTDIR=' -a '--env USER_ZDOTDIR=' -a '--env SHELL=/bin/bash' \
  -a "--env DISTROBOX_USERNAME=${USER}"
