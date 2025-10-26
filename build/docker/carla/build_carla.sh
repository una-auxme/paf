#!/bin/bash

set -e

echo "Building CARLA images locally. Only for development and testing."
echo "For building and pushing to GHCR, use build_carla_buildx.sh instead."
echo "Otherwise, pull images from <https://github.com/orgs/una-auxme/packages>"

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cd "${SCRIPT_DIR}"

if [ ! -f ./carla.tar.gz ]; then
  wget --progress=bar https://tiny.carla.org/carla-0-9-16-linux -O ./carla.tar.gz
fi
# Verify checksum
# Run "sha256sum carla.tar.gz > carla.tar.gz.sha256sum" to create new checksum
cat ./carla.tar.gz.sha256sum | sha256sum -c -

if [ ! -f ./carla-additional-maps.tar.gz ]; then
  wget --progress=bar https://tiny.carla.org/additional-maps-0-9-16-linux -O ./carla-additional-maps.tar.gz
fi
# Verify checksum
# Run "sha256sum carla.tar.gz > carla.tar.gz.sha256sum" to create new checksum
cat ./carla-additional-maps.tar.gz.sha256sum | sha256sum -c -

docker build --pull -t carla-leaderboard-gpu:2.1 --target carla -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-cuda:2.1 --target carla --build-arg BASE_FLAVOUR=cuda -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-api:2.1 --target carla-api -f ./Dockerfile ../../../
