#!/bin/bash
# This file sets up the current repo for development
set -e
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
cd "${SCRIPT_DIR}/build"

if ! grep -q PAF_USERNAME ./.env 2>/dev/null; then
  cat <<EOF >./.env
# PAF docker variables
PAF_UID=$(id -u)
PAF_GID=$(id -g)
PAF_USERNAME=$(id -u -n)
EOF
  echo Added PAF environment variables into the "${SCRIPT_DIR}/build"/.env .
  echo Docker compose will pick them up automatically.
fi

cat <<EOF

Setup successful.

Start developing by bringing up the development docker container:
- Right-click docker-compose.dev.<your-gpu-type>.yml -> Compose Up
- A Vs Code instance inside the container will open after the docker has finished building
- Make sure to install the recommended VS Code extensions
EOF
