#!/bin/bash
set -e
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cd "${SCRIPT_DIR}"

if [ ! -f ./carla.tar.gz ]; then
  wget --progress=bar https://tiny.carla.org/carla-0-9-16-linux -O ./carla.tar.gz
fi

if [ ! -f ./carla-additional-maps.tar.gz ]; then
  wget --progress=bar https://tiny.carla.org/additional-maps-0-9-16-linux -O ./carla-additional-maps.tar.gz
fi

docker build --pull -t carla-leaderboard-gpu:2.1 --target carla -f ./Dockerfile ../../../ --progress=plain
docker build --pull -t carla-leaderboard-cuda:2.1 --target carla --build-arg BASE_FLAVOUR=cuda -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-api:2.1 --target carla-api -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-ros-bridge:2.1 --target carla-ros-bridge -f ./Dockerfile ../../../
