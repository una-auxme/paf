#!/bin/bash
# Abort on error
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile"
CONTAINER_IMAGE_NAME="paf-peter-distrobox-rocm"
DISTROBOX_NAME="paf-rocm-full"

HOST_WORKSPACE_DIR="${SCRIPT_DIR}/../../../"
HOST_USER_PACKAGE_DIR="${SCRIPT_DIR}/user-package"
USER_PACKAGE_DIR=/user-package
WORKSPACE_MOUNT="/workspace"

podman build -t ${CONTAINER_IMAGE_NAME} ${SCRIPT_DIR}
distrobox create \
  --image ${CONTAINER_IMAGE_NAME} \
  --name ${DISTROBOX_NAME} \
  --home ~/Distrobox/${DISTROBOX_NAME} \
  --init-hooks '/init_hook.sh' \
  --volume ${HOST_WORKSPACE_DIR}:${WORKSPACE_MOUNT} \
  --volume ${HOST_USER_PACKAGE_DIR}:${USER_PACKAGE_DIR} \
  -a '--env ZDOTDIR=' -a '--env USER_ZDOTDIR=' -a '--env SHELL=/bin/bash' -a "--env USER_PACKAGE_DIR=${USER_PACKAGE_DIR}"
